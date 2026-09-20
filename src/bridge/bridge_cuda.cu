#include <cuda_runtime.h>

/* Host code: extern C only, for MSVC nvcc .obj linked with MinGW main app */
#define CUDA_CHECK(call) \
    do { \
        cudaError_t err = (call); \
        (void)err; \
    } while (0)

static float *d_in_x = nullptr, *d_in_y = nullptr, *d_in_z = nullptr;
static unsigned char *d_mask = nullptr;
static float *d_out_p3d_x = nullptr, *d_out_p3d_y = nullptr, *d_out_p3d_z = nullptr;
static float *d_out_cp_cx = nullptr, *d_out_cp_cy = nullptr, *d_out_cp_cz = nullptr, *d_out_cp_wcy = nullptr, *d_out_cp_habove = nullptr;
static int *d_out_count = nullptr;
static int g_max_pts = 0;
static int g_max_mask = 0;

extern "C" void freeCudaMemory(void) {
    if (d_in_x) { cudaFree(d_in_x); d_in_x = nullptr; }
    if (d_in_y) { cudaFree(d_in_y); d_in_y = nullptr; }
    if (d_in_z) { cudaFree(d_in_z); d_in_z = nullptr; }
    if (d_mask) { cudaFree(d_mask); d_mask = nullptr; }
    if (d_out_p3d_x) { cudaFree(d_out_p3d_x); d_out_p3d_x = nullptr; }
    if (d_out_p3d_y) { cudaFree(d_out_p3d_y); d_out_p3d_y = nullptr; }
    if (d_out_p3d_z) { cudaFree(d_out_p3d_z); d_out_p3d_z = nullptr; }
    if (d_out_cp_cx) { cudaFree(d_out_cp_cx); d_out_cp_cx = nullptr; }
    if (d_out_cp_cy) { cudaFree(d_out_cp_cy); d_out_cp_cy = nullptr; }
    if (d_out_cp_cz) { cudaFree(d_out_cp_cz); d_out_cp_cz = nullptr; }
    if (d_out_cp_wcy) { cudaFree(d_out_cp_wcy); d_out_cp_wcy = nullptr; }
    if (d_out_cp_habove) { cudaFree(d_out_cp_habove); d_out_cp_habove = nullptr; }
    if (d_out_count) { cudaFree(d_out_count); d_out_count = nullptr; }
    g_max_pts = 0;
    g_max_mask = 0;
}

__global__ void filterAndTransformKernel(
    const float* in_x, const float* in_y, const float* in_z, int num_points,
    float water_z, float g_min_x, float g_max_x, float g_min_y, float g_max_y, float grid_res,
    int grid_w, int grid_h, const unsigned char* mask,
    float r00, float r01, float r02, float r10, float r11, float r12, float r20, float r21, float r22,
    float tx, float ty, float tz,
    float* out_p3d_x, float* out_p3d_y, float* out_p3d_z,
    float* out_cp_cx, float* out_cp_cy, float* out_cp_cz, float* out_cp_wcy, float* out_cp_habove,
    int* out_count)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;

    float x = in_x[idx];
    float y = in_y[idx];
    float z = in_z[idx];

    if (z > water_z - 0.5f && x > g_min_x && x < g_max_x && y > g_min_y && y < g_max_y) {
        int gx = (int)((x - g_min_x) / grid_res);
        int gy = (int)((y - g_min_y) / grid_res);

        if (gx >= 0 && gx < grid_w && gy >= 0 && gy < grid_h && mask[gy * grid_w + gx]) {
            float cz = r20 * x + r21 * y + r22 * z + tz;
            if (cz < 0.1f) return;

            float cx = r00 * x + r01 * y + r02 * z + tx;
            float cy = r10 * x + r11 * y + r12 * z + ty;
            float w_cy = r10 * x + r11 * y + r12 * water_z + ty;

            int out_idx = atomicAdd(out_count, 1);
            out_p3d_x[out_idx] = x;
            out_p3d_y[out_idx] = y;
            out_p3d_z[out_idx] = z;
            out_cp_cx[out_idx] = cx;
            out_cp_cy[out_idx] = cy;
            out_cp_cz[out_idx] = cz;
            out_cp_wcy[out_idx] = w_cy;
            out_cp_habove[out_idx] = w_cy - cy;
        }
    }
}

extern "C" int runCudaFilteringAndTransform(
    const float* in_x, const float* in_y, const float* in_z, int num_points,
    float water_z, float g_min_x, float g_max_x, float g_min_y, float g_max_y, float grid_res,
    int grid_w, int grid_h, const unsigned char* mask,
    float r00, float r01, float r02, float r10, float r11, float r12, float r20, float r21, float r22,
    float tx, float ty, float tz,
    float* out_p3d_x, float* out_p3d_y, float* out_p3d_z,
    float* out_cp_cx, float* out_cp_cy, float* out_cp_cz, float* out_cp_wcy, float* out_cp_habove)
{
    if (num_points <= 0) return 0;

    if (num_points > g_max_pts) {
        if (g_max_pts > 0) freeCudaMemory();

        int alloc_pts = (int)(num_points * 1.5f);
        int pts_bytes = alloc_pts * (int)sizeof(float);

        CUDA_CHECK(cudaMalloc(&d_in_x, (size_t)pts_bytes));
        CUDA_CHECK(cudaMalloc(&d_in_y, (size_t)pts_bytes));
        CUDA_CHECK(cudaMalloc(&d_in_z, (size_t)pts_bytes));
        CUDA_CHECK(cudaMalloc(&d_out_p3d_x, (size_t)pts_bytes));
        CUDA_CHECK(cudaMalloc(&d_out_p3d_y, (size_t)pts_bytes));
        CUDA_CHECK(cudaMalloc(&d_out_p3d_z, (size_t)pts_bytes));
        CUDA_CHECK(cudaMalloc(&d_out_cp_cx, (size_t)pts_bytes));
        CUDA_CHECK(cudaMalloc(&d_out_cp_cy, (size_t)pts_bytes));
        CUDA_CHECK(cudaMalloc(&d_out_cp_cz, (size_t)pts_bytes));
        CUDA_CHECK(cudaMalloc(&d_out_cp_wcy, (size_t)pts_bytes));
        CUDA_CHECK(cudaMalloc(&d_out_cp_habove, (size_t)pts_bytes));
        CUDA_CHECK(cudaMalloc(&d_out_count, sizeof(int)));
        g_max_pts = alloc_pts;
    }

    int mask_size = grid_w * grid_h;
    if (mask_size > g_max_mask) {
        if (d_mask) CUDA_CHECK(cudaFree(d_mask));
        int alloc_mask = (int)(mask_size * 1.5f);
        CUDA_CHECK(cudaMalloc(&d_mask, (size_t)alloc_mask * sizeof(unsigned char)));
        g_max_mask = alloc_mask;
    }

    CUDA_CHECK(cudaMemset(d_out_count, 0, sizeof(int)));

    int copy_bytes = num_points * (int)sizeof(float);
    CUDA_CHECK(cudaMemcpy(d_in_x, in_x, (size_t)copy_bytes, cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_in_y, in_y, (size_t)copy_bytes, cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_in_z, in_z, (size_t)copy_bytes, cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_mask, mask, (size_t)mask_size * sizeof(unsigned char), cudaMemcpyHostToDevice));

    int threadsPerBlock = 1024;
    int blocksPerGrid = (num_points + threadsPerBlock - 1) / threadsPerBlock;

    filterAndTransformKernel<<<blocksPerGrid, threadsPerBlock>>>(
        d_in_x, d_in_y, d_in_z, num_points,
        water_z, g_min_x, g_max_x, g_min_y, g_max_y, grid_res, grid_w, grid_h, d_mask,
        r00, r01, r02, r10, r11, r12, r20, r21, r22, tx, ty, tz,
        d_out_p3d_x, d_out_p3d_y, d_out_p3d_z,
        d_out_cp_cx, d_out_cp_cy, d_out_cp_cz, d_out_cp_wcy, d_out_cp_habove, d_out_count);

    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());

    int h_count = 0;
    CUDA_CHECK(cudaMemcpy(&h_count, d_out_count, sizeof(int), cudaMemcpyDeviceToHost));

    if (h_count > 0) {
        int out_bytes = h_count * (int)sizeof(float);
        CUDA_CHECK(cudaMemcpy(out_p3d_x, d_out_p3d_x, (size_t)out_bytes, cudaMemcpyDeviceToHost));
        CUDA_CHECK(cudaMemcpy(out_p3d_y, d_out_p3d_y, (size_t)out_bytes, cudaMemcpyDeviceToHost));
        CUDA_CHECK(cudaMemcpy(out_p3d_z, d_out_p3d_z, (size_t)out_bytes, cudaMemcpyDeviceToHost));
        CUDA_CHECK(cudaMemcpy(out_cp_cx, d_out_cp_cx, (size_t)out_bytes, cudaMemcpyDeviceToHost));
        CUDA_CHECK(cudaMemcpy(out_cp_cy, d_out_cp_cy, (size_t)out_bytes, cudaMemcpyDeviceToHost));
        CUDA_CHECK(cudaMemcpy(out_cp_cz, d_out_cp_cz, (size_t)out_bytes, cudaMemcpyDeviceToHost));
        CUDA_CHECK(cudaMemcpy(out_cp_wcy, d_out_cp_wcy, (size_t)out_bytes, cudaMemcpyDeviceToHost));
        CUDA_CHECK(cudaMemcpy(out_cp_habove, d_out_cp_habove, (size_t)out_bytes, cudaMemcpyDeviceToHost));
    }

    return h_count;
}
