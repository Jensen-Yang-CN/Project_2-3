#pragma once
#include "bridge_common_types.h"
#include <deque>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include <utility>
#include <condition_variable>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <filesystem>

// PCL 依赖系统 FLANN；须先于 OpenCV，且不能 #include <flann/flann.hpp>
//（-I.../opencv2 会误包含 opencv2/flann，与 PCL 冲突）
#include <flann/general.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/transforms.h>
#include "../core/CalibrationConfig.h"
#include "../modules/LidarExtrinsic210To211.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Dense>

extern "C" int runCudaFilteringAndTransform(
    const float* in_x, const float* in_y, const float* in_z, int num_points,
    float water_z, float g_min_x, float g_max_x, float g_min_y, float g_max_y, float grid_res,
    int grid_w, int grid_h, const unsigned char* mask,
    float r00, float r01, float r02, float r10, float r11, float r12, float r20, float r21, float r22,
    float tx, float ty, float tz,
    float* out_p3d_x, float* out_p3d_y, float* out_p3d_z,
    float* out_cp_cx, float* out_cp_cy, float* out_cp_cz, float* out_cp_wcy, float* out_cp_habove);

extern "C" void freeCudaMemory(); // 引入显存释放声明

struct TrackedHole {
    float cx, width, peak_h;
    float left_cx, right_cx, top_cy, bottom_cy, avg_cz;
    int age, missed;
};

struct CamPoint {
    float cx, cy, cz, water_cy, h_above;
};

namespace usv {

    class BridgeDetector {
    private:
        cv::Mat g_K_;
        cv::Mat g_distCoeffs_;
        cv::Mat g_R_mat_;
        cv::Mat g_T_vec_;
        std::vector<TrackedHole> g_hole_tracks_;

        // 恢复：210 雷达投影到 211 雷达的外参矩阵
        Eigen::Matrix4f g_ext_210_to_211_;

        // 避免每帧分配的复用内存池
        std::vector<float> m_in_x, m_in_y, m_in_z;
        std::vector<float> m_o_px, m_o_py, m_o_pz;
        std::vector<float> m_o_cx, m_o_cy, m_o_cz, m_o_wcy, m_o_habove;

        // 异步图片保存队列。检测频率可能高于磁盘写入速度，必须限制缓存长度，
        // 否则每张高清图片都会长期占用内存，最终触发 OpenCV OOM。
        static constexpr std::size_t kMaxImageQueueSize = 4;
        std::queue<std::pair<cv::Mat, double>> m_img_queue;
        std::mutex m_queue_mutex;
        std::condition_variable m_queue_cv;
        std::thread m_writer_thread;
        std::atomic<bool> m_running;

        void imageWriterWorker() {
            while (m_running) {
                std::unique_lock<std::mutex> lock(m_queue_mutex);
                m_queue_cv.wait(lock, [this]() { return !m_img_queue.empty() || !m_running; });
                if (!m_running && m_img_queue.empty()) break;

                auto item = m_img_queue.front();
                m_img_queue.pop();
                lock.unlock();

                if (item.first.empty())
                    continue;

                const std::string save_dir = "bridge_detected_images/";
                std::error_code ec;
                std::filesystem::create_directories(save_dir, ec);

                const std::string filename = cv::format("%sbridge_detected_%.3f.jpg", save_dir.c_str(), item.second);
                cv::imwrite(filename, item.first);
            }
        }

    public:
        BridgeDetector() : m_running(false) {}

        void resetPlaybackTimeline()
        {
            g_hole_tracks_.clear();
            std::lock_guard<std::mutex> lock(m_queue_mutex);
            while (!m_img_queue.empty())
                m_img_queue.pop();
        }

        void init() {
            if (m_running)
                return;
            const BridgeCameraCalibration cam = CalibrationConfig::instance().bridgeCamera();
            g_K_ = (cv::Mat_<double>(3, 3) <<
                cam.intrinsic_k[0], cam.intrinsic_k[1], cam.intrinsic_k[2],
                cam.intrinsic_k[3], cam.intrinsic_k[4], cam.intrinsic_k[5],
                cam.intrinsic_k[6], cam.intrinsic_k[7], cam.intrinsic_k[8]);
            g_distCoeffs_ = (cv::Mat_<double>(5, 1) <<
                cam.distortion[0], cam.distortion[1], cam.distortion[2],
                cam.distortion[3], cam.distortion[4]);
            g_R_mat_ = (cv::Mat_<double>(3, 3) <<
                cam.rotation_r[0], cam.rotation_r[1], cam.rotation_r[2],
                cam.rotation_r[3], cam.rotation_r[4], cam.rotation_r[5],
                cam.rotation_r[6], cam.rotation_r[7], cam.rotation_r[8]);
            g_T_vec_ = (cv::Mat_<double>(3, 1) <<
                cam.translation_t[0], cam.translation_t[1], cam.translation_t[2]);
            g_hole_tracks_.clear();

            g_ext_210_to_211_ = lidar_extrinsic::matrix210To211Eigen4f();

            m_running = true;
            m_writer_thread = std::thread(&BridgeDetector::imageWriterWorker, this);
        }

        void shutdown() {
            if (!m_running)
                return;
            m_running = false;
            m_queue_cv.notify_all();
            if (m_writer_thread.joinable())
                m_writer_thread.join();
            freeCudaMemory();
        }

        ~BridgeDetector() {
            shutdown();
        }

        BridgeMeasureResult process(const LidarFrame& lidar_210, const LidarFrame& lidar_211, const ImageFrame& img,
                                    cv::Mat* out_ui_overlay = nullptr) {
            BridgeMeasureResult result;
            result.timestamp = (lidar_210.timestamp > 0) ? lidar_210.timestamp : lidar_211.timestamp;
            result.is_detected = false;
            result.average_center_3d = Eigen::Vector3d::Zero();
            result.width = 0.0;
            result.height = 0.0;

            pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            raw_cloud->reserve(lidar_210.points.size() + lidar_211.points.size());

            // 1. 处理 210 雷达
            // 1. 处理 210 雷达
            if (!lidar_210.points.empty()) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_210(new pcl::PointCloud<pcl::PointXYZ>);
                // 预分配最大可能空间
                cloud_210->points.resize(lidar_210.points.size());
                pcl::PointXYZ* p_cloud = cloud_210->points.data();
                const usv::LidarPoint* p_lidar = lidar_210.points.data();

                size_t valid_count = 0; // 新增：有效点计数器

                // 使用底层指针赋值加速转换，同时进行过滤
                for (size_t i = 0; i < lidar_210.points.size(); ++i) {
                    float px = p_lidar[i].x;
                    float py = p_lidar[i].y;
                    float pz = p_lidar[i].z;

                    // 【新增：剔除无效数学值及超远噪点，防止 PCL 报错】
                    if (std::isnan(px) || std::isnan(py) || std::isnan(pz)) continue;
                    if (px < -20.0f || px > 100.0f || std::abs(py) > 60.0f || pz < -5.0f || pz > 30.0f) continue;

                    // 将有效点紧凑地存放在数组前列
                    p_cloud[valid_count].x = px;
                    p_cloud[valid_count].y = py;
                    p_cloud[valid_count].z = pz;
                    valid_count++;
                }

                // 【新增：切掉数组后面未使用的预分配空间】
                cloud_210->points.resize(valid_count);
                cloud_210->width = valid_count;
                cloud_210->height = 1;

                pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_210(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::transformPointCloud(*cloud_210, *transformed_210, g_ext_210_to_211_);
                *raw_cloud += *transformed_210;
            }

            // 2. 处理 211 雷达 (400线)
            if (!lidar_211.points.empty()) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_211(new pcl::PointCloud<pcl::PointXYZ>);
                // 预分配最大可能空间
                cloud_211->points.resize(lidar_211.points.size());
                pcl::PointXYZ* p_cloud = cloud_211->points.data();
                const usv::LidarPoint* p_lidar = lidar_211.points.data();

                size_t valid_count = 0; // 新增：有效点计数器

                for (size_t i = 0; i < lidar_211.points.size(); ++i) {
                    float px = p_lidar[i].x;
                    float py = p_lidar[i].y;
                    float pz = p_lidar[i].z;

                    // 【新增：剔除无效数学值及超远噪点，防止 PCL 报错】
                    if (std::isnan(px) || std::isnan(py) || std::isnan(pz)) continue;
                    if (px < -20.0f || px > 100.0f || std::abs(py) > 60.0f || pz < -5.0f || pz > 30.0f) continue;

                    // 将有效点紧凑地存放在数组前列
                    p_cloud[valid_count].x = px;
                    p_cloud[valid_count].y = py;
                    p_cloud[valid_count].z = pz;
                    valid_count++;
                }

                // 【新增：切掉数组后面未使用的预分配空间】
                cloud_211->points.resize(valid_count);
                cloud_211->width = valid_count;
                cloud_211->height = 1;

                *raw_cloud += *cloud_211;
            }

            if (raw_cloud->empty()) {
                if (out_ui_overlay)
                    out_ui_overlay->release();
                return result;
            }

            cv::Mat display_img = img.image.clone();
            cv::Mat ui_overlay;
            if (!display_img.empty())
                ui_overlay = cv::Mat::zeros(display_img.rows, display_img.cols, CV_8UC4);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> vg_init;
            vg_init.setInputCloud(raw_cloud);
            vg_init.setLeafSize(0.1f, 0.1f, 0.1f);
            vg_init.filter(*cloud);

            pcl::PointCloud<pcl::PointXYZ>::Ptr proxy(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(cloud);
            vg.setLeafSize(0.4f, 0.4f, 0.4f);
            vg.filter(*proxy);

            pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(50); // 加入上限防止意外的长耗时解算
            seg.setDistanceThreshold(0.25);
            seg.setInputCloud(proxy);
            seg.segment(*inliers, *coeff);

            float water_z = 0.0f;
            if (!inliers->indices.empty()) {
                for (int idx : inliers->indices) water_z += proxy->points[idx].z;
                water_z /= (float)inliers->indices.size();
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr proxy_air(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& pt : proxy->points) {
                if (pt.z > water_z + 1.0f) proxy_air->push_back(pt);
            }

            bool has_bridge = false;
            pcl::PointCloud<pcl::PointXYZ>::Ptr bridge_skeleton(new pcl::PointCloud<pcl::PointXYZ>);
            if (!proxy_air->empty()) {
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
                tree->setInputCloud(proxy_air);
                std::vector<pcl::PointIndices> clusters;
                pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
                ec.setClusterTolerance(2.0); ec.setMinClusterSize(20);
                ec.setSearchMethod(tree); ec.setInputCloud(proxy_air); ec.extract(clusters);

                int best_idx = -1; float max_span = 0.0f;
                for (size_t k = 0; k < clusters.size(); ++k) {
                    float min_y = 9999.0f, max_y = -9999.0f;
                    for (int idx : clusters[k].indices) {
                        float y = proxy_air->points[idx].y;
                        min_y = (std::min)(min_y, y); max_y = (std::max)(max_y, y);
                    }
                    if (max_y - min_y > max_span) { max_span = max_y - min_y; best_idx = (int)k; }
                }
                if (best_idx != -1) {
                    for (int idx : clusters[best_idx].indices) bridge_skeleton->push_back(proxy_air->points[idx]);
                    has_bridge = true;
                }
            }

            float g_min_x = 99999.0f, g_max_x = -99999.0f, g_min_y = 99999.0f, g_max_y = -99999.0f;
            float grid_res = 0.5f; int grid_w = 1, grid_h = 1;
            std::vector<unsigned char> bridge_mask;
            if (has_bridge) {
                for (const auto& p : bridge_skeleton->points) {
                    if (p.x < g_min_x) g_min_x = p.x;
                    if (p.x > g_max_x) g_max_x = p.x;
                    if (p.y < g_min_y) g_min_y = p.y;
                    if (p.y > g_max_y) g_max_y = p.y;
                }
                g_min_x -= 2.0f; g_max_x += 2.0f; g_min_y -= 2.0f; g_max_y += 2.0f;
                grid_w = (std::max)(1, (int)std::ceil((g_max_x - g_min_x) / grid_res));
                grid_h = (std::max)(1, (int)std::ceil((g_max_y - g_min_y) / grid_res));
                bridge_mask.assign(grid_w * grid_h, 0);
                for (const auto& p : bridge_skeleton->points) {
                    int gx = (int)((p.x - g_min_x) / grid_res);
                    int gy = (int)((p.y - g_min_y) / grid_res);
                    if (gx >= 0 && gx < grid_w && gy >= 0 && gy < grid_h) bridge_mask[gy * grid_w + gx] = 1;
                }
            }

            std::vector<TrackedHole> current_frame_holes;
            if (has_bridge) {
                int n_pts = (int)cloud->points.size();

                // 复用全局内存空间，消除 vector 重复申请的巨额开销
                if (m_in_x.size() < n_pts) {
                    m_in_x.resize(n_pts * 1.2); m_in_y.resize(n_pts * 1.2); m_in_z.resize(n_pts * 1.2);
                    m_o_px.resize(n_pts * 1.2); m_o_py.resize(n_pts * 1.2); m_o_pz.resize(n_pts * 1.2);
                    m_o_cx.resize(n_pts * 1.2); m_o_cy.resize(n_pts * 1.2); m_o_cz.resize(n_pts * 1.2);
                    m_o_wcy.resize(n_pts * 1.2); m_o_habove.resize(n_pts * 1.2);
                }

                float* p_in_x = m_in_x.data(); float* p_in_y = m_in_y.data(); float* p_in_z = m_in_z.data();
                for (int i = 0; i < n_pts; i++) {
                    p_in_x[i] = cloud->points[i].x; p_in_y[i] = cloud->points[i].y; p_in_z[i] = cloud->points[i].z;
                }

                int valid_count = runCudaFilteringAndTransform(
                    p_in_x, p_in_y, p_in_z, n_pts,
                    water_z, g_min_x, g_max_x, g_min_y, g_max_y, grid_res, grid_w, grid_h, bridge_mask.data(),
                    (float)g_R_mat_.at<double>(0, 0), (float)g_R_mat_.at<double>(0, 1), (float)g_R_mat_.at<double>(0, 2),
                    (float)g_R_mat_.at<double>(1, 0), (float)g_R_mat_.at<double>(1, 1), (float)g_R_mat_.at<double>(1, 2),
                    (float)g_R_mat_.at<double>(2, 0), (float)g_R_mat_.at<double>(2, 1), (float)g_R_mat_.at<double>(2, 2),
                    (float)g_T_vec_.at<double>(0, 0), (float)g_T_vec_.at<double>(1, 0), (float)g_T_vec_.at<double>(2, 0),
                    m_o_px.data(), m_o_py.data(), m_o_pz.data(),
                    m_o_cx.data(), m_o_cy.data(), m_o_cz.data(), m_o_wcy.data(), m_o_habove.data()
                );

                float min_cx = 99999.0f, max_cx = -99999.0f;
                std::vector<CamPoint> cam_pts;
                std::vector<cv::Point3f> pts3d;
                for (int i = 0; i < valid_count; i++) {
                    CamPoint cp = { m_o_cx[i], m_o_cy[i], m_o_cz[i], m_o_wcy[i], m_o_habove[i] };
                    cam_pts.push_back(cp);
                    pts3d.push_back(cv::Point3f(m_o_cx[i], m_o_cy[i], m_o_cz[i]));

                    if (cp.cx < min_cx) min_cx = cp.cx; if (cp.cx > max_cx) max_cx = cp.cx;
                }

                if (!pts3d.empty()) {
                    if (!display_img.empty()) {
                        std::vector<cv::Point2f> pxs;
                        cv::projectPoints(pts3d, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F),
                                          g_K_, g_distCoeffs_, pxs);
                        for (const auto& p : pxs) {
                            if (p.x >= 0 && p.x < display_img.cols && p.y >= 0 && p.y < display_img.rows) {
                                cv::circle(display_img, p, 1, cv::Scalar(0, 0, 255), -1);
                                if (!ui_overlay.empty())
                                    cv::circle(ui_overlay, p, 2, cv::Scalar(0, 0, 255, 255), -1);
                            }
                        }
                    }

                    float res_x = 0.1f;
                    int num_bins = (std::max)(1, (int)std::ceil((max_cx - min_cx) / res_x));
                    std::vector<float> min_h(num_bins, 9999.0f), sum_wcy(num_bins, 0.0f), sum_cz(num_bins, 0.0f);
                    std::vector<int> cnt(num_bins, 0);
                    for (const auto& cp : cam_pts) {
                        int idx = (int)((cp.cx - min_cx) / res_x);
                        if (idx >= 0 && idx < num_bins) {
                            if (cp.h_above < min_h[idx]) min_h[idx] = cp.h_above;
                            sum_wcy[idx] += cp.water_cy; sum_cz[idx] += cp.cz; cnt[idx]++;
                        }
                    }

                    std::vector<float> ceiling(num_bins, 0.0f);
                    for (int i = 0; i < num_bins; i++) {
                        float mx = 0.0f;
                        for (int j = (std::max)(0, i - 8); j <= (std::min)(num_bins - 1, i + 8); j++)
                            if (min_h[j] != 9999.0f && min_h[j] > mx) mx = min_h[j];
                        ceiling[i] = mx;
                    }

                    int s_bin = -1;
                    for (int i = 0; i < num_bins; i++) {
                        bool is_h = (min_h[i] != 9999.0f) && (min_h[i] > 0.6f) && (ceiling[i] - min_h[i] < 0.5f);
                        if (is_h && s_bin == -1) s_bin = i;
                        else if (!is_h && s_bin != -1) {
                            int e_bin = i - 1;
                            float w = (e_bin - s_bin + 1) * res_x;
                            if (w >= 5.0f && w < 200.0f) {
                                std::vector<float> vh;
                                float twcy = 0.0f, tcz = 0.0f; int tc = 0;
                                for (int k = s_bin; k <= e_bin; k++) if (cnt[k] > 0) { vh.push_back(min_h[k]); twcy += sum_wcy[k]; tcz += sum_cz[k]; tc += cnt[k]; }
                                if (tc > 0 && !vh.empty()) {
                                    std::sort(vh.begin(), vh.end());
                                    float ph = vh[(std::min)((size_t)(vh.size() * 0.9), vh.size() - 1)];
                                    float lcx = min_cx + s_bin * res_x, rcx = min_cx + e_bin * res_x;
                                    current_frame_holes.push_back({ (lcx + rcx) / 2.0f, w, ph, lcx, rcx, (twcy / tc) - ph, (twcy / tc), (tcz / tc), 0, 0 });
                                }
                            }
                            s_bin = -1;
                        }
                    }
                    if (s_bin != -1) {
                        int e_bin = num_bins - 1;
                        float w = (e_bin - s_bin + 1) * res_x;
                        if (w >= 5.0f && w < 200.0f) {
                            std::vector<float> vh;
                            float twcy = 0.0f, tcz = 0.0f; int tc = 0;
                            for (int k = s_bin; k <= e_bin; k++) if (cnt[k] > 0) { vh.push_back(min_h[k]); twcy += sum_wcy[k]; tcz += sum_cz[k]; tc += cnt[k]; }
                            if (tc > 0 && !vh.empty()) {
                                std::sort(vh.begin(), vh.end());
                                float ph = vh[(std::min)((size_t)(vh.size() * 0.9), vh.size() - 1)];
                                float lcx = min_cx + s_bin * res_x, rcx = min_cx + e_bin * res_x;
                                current_frame_holes.push_back({ (lcx + rcx) / 2.0f, w, ph, lcx, rcx, (twcy / tc) - ph, (twcy / tc), (tcz / tc), 0, 0 });
                            }
                        }
                    }
                }
            }

            for (auto& trk : g_hole_tracks_) trk.missed++;
            for (const auto& ch : current_frame_holes) {
                bool matched = false;
                for (auto& trk : g_hole_tracks_) {
                    if (std::abs(trk.cx - ch.cx) < 2.5f) {
                        float a = (std::abs(ch.width - trk.width) / trk.width > 0.2f) ? 0.02f : 0.2f;
                        trk.left_cx = (1 - a) * trk.left_cx + a * ch.left_cx;
                        trk.right_cx = (1 - a) * trk.right_cx + a * ch.right_cx;
                        trk.top_cy = (1 - a) * trk.top_cy + a * ch.top_cy;
                        trk.bottom_cy = (1 - a) * trk.bottom_cy + a * ch.bottom_cy;
                        trk.avg_cz = (1 - a) * trk.avg_cz + a * ch.avg_cz; trk.width = trk.right_cx - trk.left_cx;
                        trk.cx = (trk.left_cx + trk.right_cx) / 2.0f; trk.peak_h = trk.bottom_cy - trk.top_cy;
                        trk.missed = 0; trk.age++; matched = true; break;
                    }
                }
                if (!matched) { TrackedHole nt = ch; nt.age = 1; nt.missed = 0; g_hole_tracks_.push_back(nt); }
            }

            for (auto it = g_hole_tracks_.begin(); it != g_hole_tracks_.end(); ) {
                if (it->missed > 3) {
                    it = g_hole_tracks_.erase(it);
                }
                else {
                    if (it->age >= 2 || it->missed == 0) {
                        std::vector<cv::Point3f> c3d = { {it->left_cx, it->top_cy, it->avg_cz}, {it->right_cx, it->top_cy, it->avg_cz},
                                                         {it->right_cx, it->bottom_cy, it->avg_cz}, {it->left_cx, it->bottom_cy, it->avg_cz} };
                        std::vector<cv::Point2f> c2d;
                        cv::projectPoints(c3d, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), g_K_, g_distCoeffs_, c2d);
                        if (!display_img.empty()) {
                            for (int e = 0; e < 4; e++) {
                                cv::line(display_img, c2d[e], c2d[(e + 1) % 4], cv::Scalar(0, 255, 0), 2);
                                if (!ui_overlay.empty())
                                    cv::line(ui_overlay, c2d[e], c2d[(e + 1) % 4], cv::Scalar(0, 255, 0, 255), 2);
                            }
                            const std::string label = cv::format("W:%.1fm P:%.1fm", it->width, it->peak_h);
                            cv::putText(display_img, label, c2d[0], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
                            if (!ui_overlay.empty())
                                cv::putText(ui_overlay, label, c2d[0], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255, 255), 2);
                        }

                        result.is_detected = true;
                        result.average_center_3d = Eigen::Vector3d(it->cx, (it->top_cy + it->bottom_cy) / 2.0, it->avg_cz);
                        result.width = it->width;
                        result.height = it->peak_h;
                    }
                    ++it;
                }
            }

            // 无相机帧（如仅回放 pcap 点云）时仍上报检测结果，但不保存空图
            if (result.is_detected && !display_img.empty()) {
                std::unique_lock<std::mutex> lock(m_queue_mutex);
                // 写盘落后时丢弃最旧图片，只保留最近的检测结果，保证内存有上限。
                while (m_img_queue.size() >= kMaxImageQueueSize)
                    m_img_queue.pop();
                m_img_queue.emplace(std::move(display_img), result.timestamp);
                lock.unlock();
                m_queue_cv.notify_one();
            }

            if (out_ui_overlay) {
                if (result.is_detected && !ui_overlay.empty())
                    *out_ui_overlay = ui_overlay.clone();
                else
                    out_ui_overlay->release();
            }

            return result;
        }
    };
} // namespace usv
