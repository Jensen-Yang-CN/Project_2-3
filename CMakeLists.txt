cmake_minimum_required(VERSION 3.18)

# Keep the compiler/CRT ABI aligned with the vcpkg PCL binaries.
set(POINTCLOUD_VCPKG_ROOT "F:/c/vcpkg" CACHE PATH "vcpkg root")
set(VCPKG_TARGET_TRIPLET "x64-mingw-dynamic-bigobj" CACHE STRING
    "vcpkg target triplet")
set(_pointcloud_vcpkg_toolchain
    "${POINTCLOUD_VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND EXISTS "${_pointcloud_vcpkg_toolchain}")
    set(CMAKE_TOOLCHAIN_FILE "${_pointcloud_vcpkg_toolchain}" CACHE FILEPATH
        "vcpkg toolchain file")
endif()

project(PointCloud LANGUAGES CXX)
option(BUILD_TESTING "Build internal regression test targets" OFF)
include(CTest)

# Keep the fixed berth library beside the built executable so Debug/Release
# packages do not depend on the original WeChat temporary-file path.
set(STATIC_BERTH_LIBRARY_JSON "泊位检测完整地理信息_20260901.json")
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${STATIC_BERTH_LIBRARY_JSON}")
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/${STATIC_BERTH_LIBRARY_JSON}"
        "${CMAKE_BINARY_DIR}/${STATIC_BERTH_LIBRARY_JSON}"
        COPYONLY)
else()
    message(WARNING
        "固定泊位 JSON 未找到：${CMAKE_CURRENT_SOURCE_DIR}/${STATIC_BERTH_LIBRARY_JSON}")
endif()

# --- 1. 全局设置 ---
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 必须开启，确保 Q_OBJECT 宏被正确处理
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
# MSYS2 GCC may not be on the parent process PATH when Qt Creator invokes
# AutoMoc. Avoid the optional compiler-predefines subprocess in that case.
set(CMAKE_AUTOMOC_COMPILER_PREDEFINES OFF)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON) # 建议关闭编译器扩展以保持标准兼容性
# 如果是 MinGW 且仍报错，可以尝试强制开启 POSIX 线程支持
if(MINGW)
    # 强制启用某些 C++17 特性需要的宏定义
    add_definitions(-D_REENTRANT)
    # 确保链接 pthread 库
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

# 允许在同一个文件中灵活配置链接目标 (解决 CMP0079)
cmake_policy(SET CMP0079 NEW)

if(WIN32)
#    add_definitions(-D_WIN32_WINNT=0x0601 -DNOMINMAX)

    # 添加 WIN32_LEAN_AND_MEAN 解决 winsock 冲突
    add_definitions(-D_WIN32_WINNT=0x0601 -DNOMINMAX -DWIN32_LEAN_AND_MEAN)
    # 新增：如果是 MinGW 编译器，屏蔽宏重定义警告
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        add_compile_options(-Wno-builtin-macro-redefined)
    endif()
endif()


# --- 2. 相对路径定义 ---
set(FFMPEG_ROOT    "${CMAKE_CURRENT_SOURCE_DIR}/external/ffmpeg-master-latest-win64-gpl-shared")
set(NPCAP_ROOT     "${CMAKE_CURRENT_SOURCE_DIR}/external/npcap-sdk-1.16")
set(LS_SDK_ROOT    "${CMAKE_CURRENT_SOURCE_DIR}/lib/LS_SDK")
set(LOCAL_LIB_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/lib")
set(BRIDGE_DIR ${CMAKE_BINARY_DIR})
# 修正：CMake 模块路径，让 find_package 能找到 FindEigen3.cmake
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.1/cmake")

# 修正：设置 Eigen3 的根目录（模块模式下也需要）
set(Eigen3_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.1")
# 也可以把根目录加入前缀路径
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.1")

# Use the same Eigen package that was used to build PCL. Fall back to the
# bundled headers only when the selected vcpkg triplet has no Eigen config.
set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.1")
set(POINTCLOUD_VCPKG_PREFIX
    "${POINTCLOUD_VCPKG_ROOT}/installed/${VCPKG_TARGET_TRIPLET}")
list(PREPEND CMAKE_PREFIX_PATH "${POINTCLOUD_VCPKG_PREFIX}")
set(_pointcloud_vcpkg_eigen_dir
    "${POINTCLOUD_VCPKG_PREFIX}/share/eigen3")
if(EXISTS "${_pointcloud_vcpkg_eigen_dir}/Eigen3Config.cmake")
    set(Eigen3_DIR "${_pointcloud_vcpkg_eigen_dir}" CACHE PATH
        "Eigen3 config directory" FORCE)
    find_package(Eigen3 CONFIG REQUIRED)
else()
    set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.1")
    if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Core")
        message(FATAL_ERROR "Eigen headers not found: ${EIGEN3_INCLUDE_DIR}")
    endif()
    if(NOT TARGET Eigen3::Eigen)
        add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
        set_target_properties(Eigen3::Eigen PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
    endif()
    set(Eigen3_FOUND TRUE)
    set(EIGEN3_FOUND TRUE)
    set(Eigen3_VERSION_STRING "3.4.1")
endif()

set(_ucrt64_prefix "D:/msys64/ucrt64" CACHE PATH "MSYS2 UCRT64 prefix")
set(OPENCV_INCLUDE_DIR "${_ucrt64_prefix}/include/opencv4")
set(OPENCV_IMPORT_DIR  "${_ucrt64_prefix}/lib")
set(OPENCV_BIN_DIR     "${_ucrt64_prefix}/bin")
# opencv2/ 为官方头文件树；链接用导入库请放在 lib/opencv/ 或构建目录（见下方 OPENCV_IMPORT_DIR）。

# --- 嵌入式 SLAM（LidarGnssSlam + NanoGICP，依赖 PCL）---
option(ENABLE_SLAM "Embed Lidar GNSS SLAM map" ON)
option(ENABLE_BRIDGE_DETECT "Embed bridge hole detection (PCL + CUDA + OpenCV)" OFF)
option(ENABLE_PCL_PERCEPTION "Enable fused-cloud bridge/pier and obstacle detection" ON)
set(SLAM_ENABLED FALSE)
set(SLAM_SRCS "")
set(BRIDGE_DETECT_ENABLED FALSE)
set(PCL_PERCEPTION_ENABLED FALSE)
set(BRIDGE_CUDA_OBJ "")
if(ENABLE_BRIDGE_DETECT OR ENABLE_SLAM OR ENABLE_PCL_PERCEPTION)
    set(POINTCLOUD_PCL_DIR "${POINTCLOUD_VCPKG_PREFIX}/share/pcl"
        CACHE PATH "PCL config directory")
    set(PCL_DIR "${POINTCLOUD_PCL_DIR}" CACHE PATH
        "PCL config directory" FORCE)
    # vcpkg 的 Debug 导入库仍使用与 Release 相同的 PCL DLL 文件名。
    # 必须按构建类型选择运行库，否则 Release DLL 会在 Debug 进程中造成跨 DLL 堆损坏。
    if(CMAKE_BUILD_TYPE STREQUAL "Debug"
       AND EXISTS "${POINTCLOUD_VCPKG_PREFIX}/debug/bin")
        set(PCL_RUNTIME_DIR "${POINTCLOUD_VCPKG_PREFIX}/debug/bin")
    else()
        set(PCL_RUNTIME_DIR "${POINTCLOUD_VCPKG_PREFIX}/bin")
    endif()
    set(_pointcloud_pcl_components
        common filters segmentation search kdtree sample_consensus octree
        features ml geometry)
    set(PCL_SLAM_COMPONENTS_AVAILABLE FALSE)
    if(EXISTS "${POINTCLOUD_VCPKG_PREFIX}/include/pcl/registration/registration.h"
       AND EXISTS "${POINTCLOUD_VCPKG_PREFIX}/include/pcl/io/pcd_io.h"
       AND EXISTS "${POINTCLOUD_VCPKG_PREFIX}/include/pcl/surface/concave_hull.h")
        set(PCL_SLAM_COMPONENTS_AVAILABLE TRUE)
        list(APPEND _pointcloud_pcl_components registration io surface)
    endif()
    find_package(PCL 1.15 CONFIG QUIET
        COMPONENTS ${_pointcloud_pcl_components})

    if(PCL_FOUND)
        message(STATUS "PCL ${PCL_VERSION} from ${PCL_DIR}")
        message(STATUS "  includes=${PCL_INCLUDE_DIRS}")
        message(STATUS "  libraries=${PCL_LIBRARIES}")
    else()
        message(WARNING
            "PCL not found in ${PCL_DIR}; expected triplet "
            "${VCPKG_TARGET_TRIPLET}")
    endif()

    if(NOT PCL_SLAM_COMPONENTS_AVAILABLE AND ENABLE_SLAM)
        message(WARNING
            "SLAM requires PCL registration/io/surface, but the installed "
            "${VCPKG_TARGET_TRIPLET} package only contains the base modules")
    endif()

    # --- CUDA：不调用 enable_language(CUDA)，避免 CMakeCUDACompilerId.cu 在 MinGW 下失败 ---
    # 改用 nvcc -ccbin g++ 手动编译 bridge_cuda.cu
    if(DEFINED ENV{CUDA_PATH})
        set(CUDAToolkit_ROOT "$ENV{CUDA_PATH}" CACHE PATH "CUDA toolkit root")
    elseif(EXISTS "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v11.6")
        set(CUDAToolkit_ROOT "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v11.6" CACHE PATH "CUDA toolkit root")
    endif()

    find_program(_NVCC_EXECUTABLE NAMES nvcc nvcc.exe
        HINTS "${CUDAToolkit_ROOT}/bin" "$ENV{CUDA_PATH}/bin"
        PATHS "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v11.6/bin")

    # Windows：nvcc 官方只认 MSVC，不能用 MinGW g++（会报 Host compiler targets unsupported OS）
    # bridge_cuda.cu 导出为 extern "C"，可用 MSVC 编 .cu，再用 MinGW 链接进主程序
    set(_NVCC_HOST_CCBIN "")
    set(_NVCC_VCVARS_BAT "")
    set(_NVCC_VCVARS_VER "")
    if(WIN32 AND _NVCC_EXECUTABLE)
        # VS 2026 默认：18\Community，cl 在 VC\Tools\MSVC\*\bin\Hostx64\x64\cl.exe
        set(VS_INSTALL_DIR "C:/Program Files/Microsoft Visual Studio/18/Community" CACHE PATH
            "Visual Studio 安装目录（含 VC/Auxiliary/Build/vcvars64.bat）")

        find_program(_VSWHERE_EXECUTABLE NAMES vswhere vswhere.exe
            PATHS
                "C:/Program Files (x86)/Microsoft Visual Studio/Installer"
                "C:/Program Files/Microsoft Visual Studio/Installer"
            NO_DEFAULT_PATH)

        set(_VS_INSTALL_DIR "${VS_INSTALL_DIR}")
        if(NOT _VS_INSTALL_DIR AND _VSWHERE_EXECUTABLE)
            foreach(_vswhere_extra "-prerelease" "")
                execute_process(
                    COMMAND "${_VSWHERE_EXECUTABLE}" -latest ${_vswhere_extra} "-products" "*"
                        -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64
                        -property installationPath
                    OUTPUT_VARIABLE _VS_INSTALL_DIR
                    OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_QUIET)
                if(_VS_INSTALL_DIR)
                    break()
                endif()
            endforeach()
        endif()

        if(NOT _VS_INSTALL_DIR)
            foreach(_vs_guess
                "C:/Program Files/Microsoft Visual Studio/18/Community"
                "C:/Program Files/Microsoft Visual Studio/2026/Community"
                "C:/Program Files/Microsoft Visual Studio/2022/Community"
                "C:/Program Files/Microsoft Visual Studio/2022/BuildTools")
                if(EXISTS "${_vs_guess}/VC/Auxiliary/Build/vcvars64.bat")
                    set(_VS_INSTALL_DIR "${_vs_guess}")
                    break()
                endif()
            endforeach()
        endif()

        if(_VS_INSTALL_DIR AND EXISTS "${_VS_INSTALL_DIR}/VC/Auxiliary/Build/vcvars64.bat")
            set(_NVCC_VCVARS_BAT "${_VS_INSTALL_DIR}/VC/Auxiliary/Build/vcvars64.bat")
            file(GLOB _MSVC_CL_LIST
                "${_VS_INSTALL_DIR}/VC/Tools/MSVC/*/bin/Hostx64/x64/cl.exe"
                "${_VS_INSTALL_DIR}/VC/Tools/MSVC/*/bin/HostX64/x64/cl.exe")
            list(SORT _MSVC_CL_LIST)
            if(_MSVC_CL_LIST)
                list(GET _MSVC_CL_LIST -1 _NVCC_CL_EXE)
            else()
                find_program(_NVCC_CL_EXE NAMES cl.exe
                    PATHS "${_VS_INSTALL_DIR}/VC/Tools/MSVC"
                    PATH_SUFFIXES "14.44.35207/bin/Hostx64/x64"
                                  "14.44.35207/bin/HostX64/x64"
                    NO_DEFAULT_PATH)
            endif()
            if(_NVCC_CL_EXE)
                get_filename_component(_NVCC_HOST_CCBIN "${_NVCC_CL_EXE}" DIRECTORY)
                if(_NVCC_CL_EXE MATCHES "/MSVC/([0-9]+\\.[0-9]+)/")
                    set(_NVCC_VCVARS_VER "${CMAKE_MATCH_1}")
                endif()
                message(STATUS "Bridge detect: VS=${_VS_INSTALL_DIR}")
                message(STATUS "Bridge detect: MSVC cl=${_NVCC_CL_EXE}")
            else()
                message(WARNING "Bridge detect: 已找到 vcvars64，但未找到 cl.exe，请检查 VC/Tools/MSVC/*/bin/Hostx64/x64/")
            endif()
        endif()
    endif()

    set(_bridge_cuda_ok FALSE)
    if(_NVCC_EXECUTABLE AND _NVCC_HOST_CCBIN AND _NVCC_VCVARS_BAT)
        set(BRIDGE_CUDA_OBJ "${CMAKE_BINARY_DIR}/bridge_cuda.obj")
        set(_bridge_cuda_ok TRUE)
        message(STATUS "Bridge detect: nvcc 编译 CUDA（主机编译器=MSVC cl，与 MinGW 主工程链接 extern C）")
        message(STATUS "  nvcc=${_NVCC_EXECUTABLE}")
        message(STATUS "  ccbin=${_NVCC_HOST_CCBIN}")
        message(STATUS "  vcvars=${_NVCC_VCVARS_BAT} (ver=${_NVCC_VCVARS_VER})")
    elseif(_NVCC_EXECUTABLE AND CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        set(BRIDGE_CUDA_OBJ "${CMAKE_BINARY_DIR}/bridge_cuda.obj")
        set(_NVCC_HOST_CCBIN "${CMAKE_CXX_COMPILER}")
        set(_bridge_cuda_ok TRUE)
        message(STATUS "Bridge detect: nvcc -ccbin MSVC（全工程 MSVC Kit）")
    elseif(_NVCC_EXECUTABLE)
        message(WARNING
            "Bridge detect: 已找到 nvcc，但未找到 Visual Studio C++ (cl.exe)。"
            " Windows 上 CUDA 不能用 MinGW g++ 编译 .cu，请安装 VS Build Tools。")
    else()
        message(WARNING "Bridge detect: 未找到 nvcc，请安装 CUDA 11.6 或设置 CUDA_PATH。")
    endif()

    if(ENABLE_BRIDGE_DETECT AND PCL_FOUND AND _bridge_cuda_ok)
        set(BRIDGE_DETECT_ENABLED TRUE)
        message(STATUS "Bridge detect: 预检通过（PCL + 手动 nvcc）；Configure 完成后需 OpenCV 导入库")
    else()
        set(BRIDGE_DETECT_ENABLED FALSE)
        if(NOT PCL_FOUND AND NOT _bridge_cuda_ok)
            message(WARNING "Bridge detect: OFF — 需要 PCL 与 CUDA(nvcc+g++)。")
        elseif(NOT PCL_FOUND)
            message(WARNING "Bridge detect: OFF — 仅缺 PCL（nvcc+g++ 已就绪）。")
        else()
            message(WARNING "Bridge detect: OFF — 仅缺 nvcc/g++ 配置。")
        endif()
    endif()

    if(ENABLE_SLAM AND PCL_FOUND AND PCL_SLAM_COMPONENTS_AVAILABLE)
        set(SLAM_ENABLED TRUE)
        set(SLAM_SRCS
            src/slam/DloSlamNode.h
            src/slam/DloSlamNode.cpp
            src/slam/SlamGeoAnchorPolicy.h
            src/slam/SlamMapGeoUtils.h
            src/slam/SlamMapGeoUtils.cpp
            src/slam/SlamMapGeoCorrector.h
            src/slam/SlamMapGeoCorrector.cpp
            src/slam/SlamMapBerthGeometry.h
            src/slam/SlamMapStitcher.h
            src/slam/SlamMapStitcher.cpp
            src/slam/SlamMapLoadTask.h
            src/slam/SlamMapLoadTask.cpp
            src/slam/SlamTileMapTypes.h
            src/slam/SlamTileMapIO.h
            src/slam/SlamTileMapIO.cpp
            src/slam/SlamTileMapBuilder.h
            src/slam/SlamTileMapBuilder.cpp
            src/slam/SlamTileMapManager.h
            src/slam/SlamTileMapManager.cpp
            src/slam/SlamMapDisplayBuilder.h
            src/slam/SlamMapDisplayBuilder.cpp
            src/slam/WaterSurfaceFilter.h
            src/slam/WaterSurfaceFilter.cpp
            src/bus/HistoricalMapExportSource.h
            src/bus/HistoricalMapExportSource.cpp
            src/code-slam/LidarGnssSlamConfig.h
            src/code-slam/LidarGnssSlam.h
            src/code-slam/LidarGnssSlam.cpp
            src/code-slam/nano_gicp/nano_gicp.h
            src/code-slam/nano_gicp/nano_gicp.cc
            src/code-slam/nano_gicp/lsq_registration.h
            src/code-slam/nano_gicp/lsq_registration.cc
            src/code-slam/nano_gicp/nanoflann.h
            src/code-slam/nano_gicp/nanoflann.cc
            src/code-slam/nano_gicp/nanoflann_adaptor.h
            src/code-slam/nano_gicp/nano_gicp_types.h
            src/code-slam/nano_gicp/gicp_utils.h
        )
        message(STATUS "SLAM: enabled (LidarGnssSlam + DloSlamNode)")
    elseif(ENABLE_SLAM)
        message(WARNING
            "SLAM: OFF — 当前 vcpkg PCL 缺少 registration/io/surface 模块")
    endif()

    if(ENABLE_PCL_PERCEPTION AND PCL_FOUND)
        set(PCL_PERCEPTION_ENABLED TRUE)
        message(STATUS "PCL perception: enabled (fused-cloud bridge/pier/obstacle)")
    elseif(ENABLE_PCL_PERCEPTION)
        message(WARNING "PCL perception: disabled because PCL was not found")
    endif()
endif()

if(BUILD_TESTING AND SLAM_ENABLED)
    add_executable(water_surface_filter_tests
        tests/water_surface_filter_tests.cpp
        src/slam/WaterSurfaceFilter.cpp
        src/slam/WaterSurfaceFilter.h
        common/pointxyz.h
    )
    target_include_directories(water_surface_filter_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/common
        ${CMAKE_CURRENT_SOURCE_DIR}/src/slam
    )
    target_link_libraries(water_surface_filter_tests PRIVATE
        Qt5::Core
        Eigen3::Eigen
    )
    add_test(NAME water_surface_filter
        COMMAND water_surface_filter_tests)

endif()

# --- 3. 查找依赖 ---
find_package(Qt5 COMPONENTS Widgets Core Network Concurrent OpenGL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)
if(NOT TARGET Eigen3::Eigen)
    find_package(Eigen3 REQUIRED)
endif()

## --- 4. 协议解析对象库 (OBJECT Library) ---
## 先定义，再配置
add_library(protocol_objs OBJECT)

target_sources(protocol_objs PRIVATE
    src/protocol/rtpparser.cpp      src/protocol/rtpparser.h
    src/protocol/RtpParserH264.cpp  src/protocol/RtpParserH264.h
    src/protocol/RsLidarParser.cpp  src/protocol/RsLidarParser.h
    src/protocol/Ls400Parser.cpp    src/protocol/Ls400Parser.h
    src/protocol/IMUParser.cpp      src/protocol/IMUParser.h
    src/protocol/LidarPacketStruct.h
    src/protocol/NaviProtocol.h
)

## 关键修复 4.1：解决 IMUParser.h 找不到 QObject 的问题

## 为对象库添加必要的宏定义
target_compile_definitions(protocol_objs PRIVATE
    WPCAP
    HAVE_REMOTE
    _CRT_SECURE_NO_WARNINGS
)
# 4.2必须直接给对象库链接 Qt 模块，它才能拿到 Qt 的头文件路径
target_link_libraries(protocol_objs PUBLIC
    Qt5::Core
    Qt5::Network
)

## 4.3设置对象库的包含路径
message(STATUS "NPCAP PATH: ${NPCAP_ROOT}/include")
target_include_directories(protocol_objs PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/protocol
    ${CMAKE_CURRENT_SOURCE_DIR}/rs_driver/include
    ${CMAKE_CURRENT_SOURCE_DIR}/rs_driver
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${NPCAP_ROOT}/include
    ${NPCAP_ROOT}/include/pcap
    ${LS_SDK_ROOT}/include
)

# --- 5. 源文件分组定义 ---
set(APP_SRCS
    src/main.cpp
    src/res.qrc
)

set(CORE_SRCS
    src/core/BaseWorker.h
    src/core/QDebugStream.h
    src/core/AppConfig.h
    src/core/AppConfig.cpp
    src/core/SystemLogConfig.h
    src/core/SystemFileLogger.h
    src/core/SystemFileLogger.cpp
    src/core/SensorMetrics.h
    src/core/SensorMetrics.cpp
    src/core/SystemLogHelpers.h
    src/core/SystemLogHelpers.cpp
    src/core/CalibrationConfig.h
    src/core/CalibrationConfig.cpp
    src/core/SlamMapBinaryIO.h
    src/core/SlamMapBinaryIO.cpp
)

set(NETWORK_SRCS
    src/network/UdpDispatcher.h      src/network/UdpDispatcher.cpp
    src/network/PcapPlaybackController.h src/network/PcapPlaybackController.cpp
    src/network/PcapPlaybackRequest.h
    src/network/PcapPlaybackTime.h src/network/PcapPlaybackTime.cpp
    src/network/PcapngReader.h       src/network/PcapngReader.cpp
    src/network/LivePcapCapture.h    src/network/LivePcapCapture.cpp
    src/network/UdpSocketReceiver.h  src/network/UdpSocketReceiver.cpp
    src/network/PcapDeviceDialog.h   src/network/PcapDeviceDialog.cpp
)

set(DEBUG_SRCS
    src/debug/PauseSnapshot.h src/debug/PauseSnapshot.cpp
)

set(COMMON_SRCS
    common/pointxyz.h
    common/types.h
    common/slam_types.h
)

set(BRIDGE_SRCS
    src/bridge/bridge_common_types.h
    src/bridge/BridgeDetector.h
    src/bridge/TraditionalBridgeVisionDetector.h
    src/bridge/bridge_cuda.cu
)

set(BERTH_SRCS
    src/berth/BerthDetection.h
    src/berth/BerthOpeningRules.h
    src/berth/BerthRectGeometry.h
    src/berth/BerthRecoveryRules.h
    src/berth/LineBerthDetection.h
)

set(BUS_SRCS
    src/bus/CommunicationManager_v2.h
    src/bus/ThreadSafeQueue.h
    src/bus/common_types_extended.h
    src/bus/bus_types.h
    src/bus/PreProcessorNode_v2.h
    src/bus/SensorPublisherNode.h  src/bus/SensorPublisherNode.cpp
    src/bus/BridgeDetectionNode.h       src/bus/BridgeDetectionNode.cpp
    src/bus/BerthDetectionNode.h        src/bus/BerthDetectionNode.cpp
    src/bus/BerthFusionRules.h
    src/bus/DetectionNodeScheduler.h
    src/bus/CommBusManager.h             src/bus/CommBusManager.cpp
    src/bus/bus_convert.h                src/bus/bus_convert.cpp
    src/bus/PerceptionExportNode.h       src/bus/PerceptionExportNode.cpp
    src/bus/GnssInsGeoUtils.h            src/bus/GnssInsGeoUtils.cpp
    src/bus/PerceptionUdpProtocol.h
    src/bus/BerthUdpPacketBuilder.h       src/bus/BerthUdpPacketBuilder.cpp
    src/bus/StaticBerthLibrary.h           src/bus/StaticBerthLibrary.cpp
    src/bus/BerthProtocolGeometry.h
    src/bus/BerthGeometryTransform.h
    src/bus/BerthOccupancy.h
    src/bus/BerthPoseBuffer.h
    src/bus/RealtimeBerthCoordinateLibrary.h
    src/bus/RealtimeBerthRecall.h
    src/bus/RealtimeBerthStability.h
    src/bus/RealtimeWorldAnchorBus.h
)

set(MODULES_SRCS
    src/modules/ImuWorker.h        src/modules/ImuWorker.cpp
    src/modules/VideoWorker.h      src/modules/VideoWorker.cpp
    src/modules/RsLidarWorker.h    src/modules/RsLidarWorker.cpp
    src/modules/Ls400Worker.h      src/modules/Ls400Worker.cpp
    src/modules/AVDecoder.h        src/modules/AVDecoder.cpp
    src/modules/RtspPullWorker.h   src/modules/RtspPullWorker.cpp
    src/modules/radarfusionmanager.h src/modules/radarfusionmanager.cpp
    src/modules/LidarExtrinsic210To211.h
    src/modules/RadarFusionPolicy.h
    src/modules/radartransform.h
    src/modules/devicemanager.h    src/modules/devicemanager.cpp
    src/modules/DeviceConfig.h
    src/modules/bridge_detection_worker.h
    src/modules/bridge_detection_worker.cpp
)

set(GUI_SRCS
    src/gui/mainwindow.h           src/gui/mainwindow.cpp       src/gui/mainwindow.ui
    src/gui/videowidget.h          src/gui/videowidget.cpp
    src/gui/imuwidget.h            src/gui/imuwidget.cpp
    src/gui/datapanel.h            src/gui/datapanel.cpp        src/gui/datapanel.ui
    src/gui/MultiLidarWidget.h     src/gui/MultiLidarWidget.cpp
    src/gui/BerthOverlayStyle.h
    src/gui/SlamBerthOverlaySynchronizer.h
    src/slam/SlamMapBerthGeometry.h
    src/slam/SlamMapBerthStore.h
    src/gui/adddevicedialog.h      src/gui/adddevicedialog.cpp  src/gui/adddevicedialog.ui
    src/gui/dialogdevicemanager.h  src/gui/dialogdevicemanager.cpp src/gui/dialogdevicemanager.ui
)
#slam地图相关
set(_slam_ide_srcs "")
if(NOT SLAM_ENABLED)
    set(_slam_ide_srcs
        src/slam/DloSlamNode.h
        src/code-slam/LidarGnssSlam.h
        src/code-slam/nano_gicp/nano_gicp.h
        src/code-slam/nano_gicp/lsq_registration.h
        src/code-slam/nano_gicp/nanoflann.h
        src/code-slam/nano_gicp/nanoflann_adaptor.h
        src/code-slam/nano_gicp/nano_gicp_types.h
        src/code-slam/nano_gicp/gicp_utils.h
    )
endif()

# --- 6. 创建执行程序 ---
add_executable(${PROJECT_NAME} WIN32
    ${APP_SRCS}
    ${COMMON_SRCS}
    ${CORE_SRCS}
    ${NETWORK_SRCS}
    ${DEBUG_SRCS}
    ${BUS_SRCS}
    ${BERTH_SRCS}
    ${BRIDGE_SRCS}
    ${MODULES_SRCS}
    ${GUI_SRCS}
    ${_slam_ide_srcs}
    ${SLAM_SRCS}
    $<TARGET_OBJECTS:protocol_objs>
)

if(PCL_PERCEPTION_ENABLED)
    target_sources(${PROJECT_NAME} PRIVATE
        src/modules/pcl_cluster_preview.h
        src/modules/pcl_cluster_preview.cpp
    )
    set_source_files_properties(
        "${CMAKE_CURRENT_SOURCE_DIR}/src/modules/pcl_cluster_preview.cpp"
        PROPERTIES
            INCLUDE_DIRECTORIES "${PCL_INCLUDE_DIRS}"
            COMPILE_DEFINITIONS "PCL_SILENCE_MALLOC_WARNING=1")
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        ENABLE_PCL_PERCEPTION=1)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES})
endif()

# bridge_cuda.cu 由下方 nvcc 自定义命令编译；此处仅用于 IDE 展示，不参与 C++ 编译
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.26")
    set_source_files_properties(
        "${CMAKE_CURRENT_SOURCE_DIR}/src/bridge/bridge_cuda.cu"
        PROPERTIES HEADER_FILE_ONLY TRUE)
endif()
if(NOT SLAM_ENABLED)
    # These files are listed only so IDEs can display them. DloSlamNode has
    # Q_OBJECT and must also be excluded from AutoMoc when PCL is unavailable.
    set_source_files_properties(
        ${_slam_ide_srcs}
        PROPERTIES
            HEADER_FILE_ONLY TRUE
            SKIP_AUTOMOC TRUE)
endif()

# --- 7. 包含路径 (Include Directories) ---
target_include_directories(${PROJECT_NAME} PRIVATE
    src/core
    src/network
    src/debug
    src/protocol
    src/modules
    src/bridge
    src/berth
    src/bus
    src/gui
    src/slam
    src/code-slam
    src/code-slam/nano_gicp
    common
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${FFMPEG_ROOT}/include
    ${NPCAP_ROOT}/include
    ${LS_SDK_ROOT}/include
    ${NPCAP_ROOT}/include
    ${NPCAP_ROOT}/include/pcap
    ${EIGEN3_DIR}
    ${OPENCV_INCLUDE_DIR}
    ${BRIDGE_DIR}
)

# --- 8. 库搜索路径 (Link Directories) ---
target_link_directories(${PROJECT_NAME} PRIVATE
    ${FFMPEG_ROOT}/lib
    ${NPCAP_ROOT}/lib
    ${LS_SDK_ROOT}/lib
    ${LOCAL_LIB_DIR}


)


# --- 9. 链接库 (Link Libraries) ---
target_link_libraries(PointCloud PRIVATE
    Qt5::Widgets
    Qt5::Core
    Qt5::Network
    Qt5::Concurrent
    ${OPENGL_LIBRARIES}
    # FFmpeg 库
    ${LOCAL_LIB_DIR}/libavformat.dll.a
    ${LOCAL_LIB_DIR}/libavcodec.dll.a
    ${LOCAL_LIB_DIR}/libswscale.dll.a
    ${LOCAL_LIB_DIR}/libswresample.dll.a
    ${LOCAL_LIB_DIR}/libavutil.dll.a
    # 网络与捕获库 (rs_driver 核心依赖)
    ${LOCAL_LIB_DIR}/libwpcap.a
    ${LOCAL_LIB_DIR}/libPacket.a
    ws2_32
    mswsock            # 新增：Boost.Asio 可能需要
    iphlpapi           # 新增：网络接口查询可能需要
    #LS400解码库
    ${LOCAL_LIB_DIR}/libGetLidarData_LS.dll.a
    # 线程库 (rs_driver 是多线程的)
    Threads::Threads
    Eigen3::Eigen
 )

# --- OpenCV: use MSYS2/ucrt64 OpenCV ---
file(GLOB OCV_IMPORT_LIBS "${OPENCV_IMPORT_DIR}/libopencv_*.dll.a")
file(GLOB OCV_RUNTIME_DLLS "${OPENCV_BIN_DIR}/libopencv_*.dll")

if(OCV_IMPORT_LIBS)
    list(SORT OCV_IMPORT_LIBS)

    message(STATUS "OpenCV include: ${OPENCV_INCLUDE_DIR}")
    message(STATUS "OpenCV import dir: ${OPENCV_IMPORT_DIR}")
    message(STATUS "OpenCV modules: ${OCV_IMPORT_LIBS}")

    target_link_directories(${PROJECT_NAME} PRIVATE "${OPENCV_IMPORT_DIR}")
    target_link_libraries(${PROJECT_NAME} PRIVATE ${OCV_IMPORT_LIBS})
    target_compile_definitions(${PROJECT_NAME} PRIVATE POINTCLOUD_LINKS_OPENCV)

    if(BRIDGE_DETECT_ENABLED)
        message(STATUS "Bridge detect: OpenCV import libs OK")
    endif()

    if(OCV_RUNTIME_DLLS)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${OCV_RUNTIME_DLLS}
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
            COMMENT "Copying OpenCV DLLs to executable directory..."
        )
    endif()
else()
    message(WARNING "OpenCV disabled: no libopencv_*.dll.a found in ${OPENCV_IMPORT_DIR}")
endif()

if(BUILD_TESTING)
    add_executable(radar_update_tests
        tests/radar_update_tests.cpp
        src/modules/RadarFusionPolicy.h
        src/core/AppConfig.h
        src/core/AppConfig.cpp
        src/core/CalibrationConfig.h
        src/core/CalibrationConfig.cpp
    )
    target_include_directories(radar_update_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/core
        ${CMAKE_CURRENT_SOURCE_DIR}/src/modules
        ${EIGEN3_INCLUDE_DIR}
    )
    target_link_libraries(radar_update_tests PRIVATE
        Qt5::Core
        Qt5::Gui
        Eigen3::Eigen
    )
    add_test(NAME radar_update COMMAND radar_update_tests)

    add_executable(radar_fusion_output_tests
        tests/radar_fusion_output_tests.cpp
        src/modules/radarfusionmanager.cpp
        src/modules/radarfusionmanager.h
        src/modules/RadarFusionPolicy.h
        src/modules/radartransform.h
        src/core/AppConfig.cpp
        src/core/AppConfig.h
        src/core/CalibrationConfig.cpp
        src/core/CalibrationConfig.h
        common/pointxyz.h
    )
    target_include_directories(radar_fusion_output_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/common
        ${CMAKE_CURRENT_SOURCE_DIR}/src/core
        ${CMAKE_CURRENT_SOURCE_DIR}/src/modules
        ${EIGEN3_INCLUDE_DIR}
    )
    target_link_libraries(radar_fusion_output_tests PRIVATE
        Qt5::Core
        Qt5::Gui
        Eigen3::Eigen
    )
    add_test(NAME radar_fusion_output COMMAND radar_fusion_output_tests)

    add_executable(slam_pose_tests
        tests/slam_pose_tests.cpp
        src/slam/SlamPoseUtils.h
        src/bridge/bridge_common_types.h
    )
    target_include_directories(slam_pose_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/common
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge
        ${CMAKE_CURRENT_SOURCE_DIR}/src/slam
        ${EIGEN3_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
    )
    target_link_libraries(slam_pose_tests PRIVATE
        Qt5::Core
        Eigen3::Eigen
    )
    add_test(NAME slam_pose COMMAND slam_pose_tests)

    add_executable(slam_display_tests
        tests/slam_display_tests.cpp
        src/gui/SlamDisplayPolicy.h
        src/gui/UsvDirectionUtils.h
    )
    target_include_directories(slam_display_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gui
    )
    add_test(NAME slam_display COMMAND slam_display_tests)

    add_executable(slam_berth_overlay_tests
        tests/slam_berth_overlay_tests.cpp
    )
    target_include_directories(slam_berth_overlay_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bus
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gui
        ${CMAKE_CURRENT_SOURCE_DIR}/src/slam
        ${EIGEN3_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
    )
    target_link_libraries(slam_berth_overlay_tests PRIVATE
        Qt5::Core
        Eigen3::Eigen
    )
    add_test(NAME slam_berth_overlay COMMAND slam_berth_overlay_tests)

    add_executable(slam_map_berth_store_tests
        tests/slam_map_berth_store_tests.cpp
        src/slam/SlamMapBerthGeometry.h
        src/slam/SlamMapBerthStore.h
        common/slam_types.h
    )
    target_include_directories(slam_map_berth_store_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/common
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bus
        ${CMAKE_CURRENT_SOURCE_DIR}/src/slam
        ${EIGEN3_INCLUDE_DIR}
    )
    target_link_libraries(slam_map_berth_store_tests PRIVATE
        Qt5::Core
        Eigen3::Eigen
    )
    add_test(NAME slam_map_berth_store COMMAND slam_map_berth_store_tests)

    add_executable(slam_map_io_tests
        tests/slam_map_io_tests.cpp
        src/core/SlamMapBinaryIO.h
        src/core/SlamMapBinaryIO.cpp
        src/core/AppConfig.h
        src/core/AppConfig.cpp
        common/slam_types.h
        common/pointxyz.h
    )
    target_include_directories(slam_map_io_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/common
        ${CMAKE_CURRENT_SOURCE_DIR}/src/core
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge
        ${EIGEN3_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
    )
    target_link_libraries(slam_map_io_tests PRIVATE
        Qt5::Core
        Eigen3::Eigen
    )
    add_test(NAME slam_map_io COMMAND slam_map_io_tests)

    add_executable(slam_map_source_merge_tests
        tests/slam_map_source_merge_tests.cpp
    )
    add_test(NAME slam_map_source_merge COMMAND slam_map_source_merge_tests)

    add_executable(slam_tile_cache_tests
        tests/slam_tile_cache_tests.cpp
        src/slam/SlamTileMapTypes.h
        src/slam/SlamTileMapIO.h
        src/slam/SlamTileMapIO.cpp
        src/slam/SlamTileMapManager.h
        src/slam/SlamTileMapManager.cpp
    )
    target_include_directories(slam_tile_cache_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/common
        ${CMAKE_CURRENT_SOURCE_DIR}/src/slam
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge
        ${EIGEN3_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
    )
    target_link_libraries(slam_tile_cache_tests PRIVATE
        Qt5::Core
        Qt5::Concurrent
    )
    add_test(NAME slam_tile_cache COMMAND slam_tile_cache_tests)

    add_executable(slam_tile_map_tests
        tests/slam_tile_map_tests.cpp
        src/slam/SlamTileMapTypes.h
        src/slam/SlamTileMapIO.h
        src/slam/SlamTileMapIO.cpp
        src/slam/SlamTileMapBuilder.h
        src/slam/SlamTileMapBuilder.cpp
    )
    target_include_directories(slam_tile_map_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/common
        ${CMAKE_CURRENT_SOURCE_DIR}/src/core
        ${CMAKE_CURRENT_SOURCE_DIR}/src/slam
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge
        ${EIGEN3_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
    )
    target_link_libraries(slam_tile_map_tests PRIVATE
        Qt5::Core
        Eigen3::Eigen
    )
    add_test(NAME slam_tile_map COMMAND slam_tile_map_tests)

    add_executable(historical_map_export_tests
        tests/historical_map_export_tests.cpp
        src/bus/HistoricalMapExportSource.h
        src/bus/HistoricalMapExportSource.cpp
        src/slam/SlamMapGeoUtils.h
        src/slam/SlamMapGeoUtils.cpp
        src/slam/SlamTileMapTypes.h
        src/slam/SlamTileMapIO.h
        src/slam/SlamTileMapIO.cpp
    )
    target_include_directories(historical_map_export_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/common
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bus
        ${CMAKE_CURRENT_SOURCE_DIR}/src/slam
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge
        ${EIGEN3_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
    )
    target_link_libraries(historical_map_export_tests PRIVATE
        Qt5::Core
        Eigen3::Eigen
    )
    add_test(NAME historical_map_export COMMAND historical_map_export_tests)

    add_executable(slam_tile_config_tests
        tests/slam_tile_config_tests.cpp
        src/core/AppConfig.h
        src/core/AppConfig.cpp
    )
    target_include_directories(slam_tile_config_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/core
    )
    target_link_libraries(slam_tile_config_tests PRIVATE Qt5::Core)
    add_test(NAME slam_tile_config COMMAND slam_tile_config_tests)

    if(SLAM_ENABLED)
        add_executable(slam_map_stitching_tests
            tests/slam_map_stitching_tests.cpp
            src/slam/SlamMapGeoUtils.h
            src/slam/SlamMapGeoUtils.cpp
            src/slam/SlamMapGeoCorrector.h
            src/slam/SlamMapGeoCorrector.cpp
            src/slam/SlamMapBerthGeometry.h
            src/slam/SlamMapStitcher.h
            src/slam/SlamMapStitcher.cpp
            src/slam/SlamMapLoadTask.h
            src/slam/SlamMapLoadTask.cpp
            src/slam/SlamTileMapTypes.h
            src/slam/SlamTileMapIO.h
            src/slam/SlamTileMapIO.cpp
            src/slam/SlamTileMapBuilder.h
            src/slam/SlamTileMapBuilder.cpp
            src/slam/SlamMapDisplayBuilder.h
            src/slam/SlamMapDisplayBuilder.cpp
            src/core/SlamMapBinaryIO.h
            src/core/SlamMapBinaryIO.cpp
            src/core/AppConfig.h
            src/core/AppConfig.cpp
            common/slam_types.h
            common/pointxyz.h
        )
        target_include_directories(slam_map_stitching_tests PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/common
            ${CMAKE_CURRENT_SOURCE_DIR}/src/core
            ${CMAKE_CURRENT_SOURCE_DIR}/src/slam
            ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge
            ${EIGEN3_INCLUDE_DIR}
            ${OPENCV_INCLUDE_DIR}
            ${PCL_INCLUDE_DIRS}
        )
        target_compile_definitions(slam_map_stitching_tests PRIVATE
            PCL_SILENCE_MALLOC_WARNING=1
        )
        target_link_libraries(slam_map_stitching_tests PRIVATE
            Qt5::Core
            Eigen3::Eigen
            ${PCL_LIBRARIES}
        )
        add_test(NAME slam_map_stitching COMMAND slam_map_stitching_tests)
    endif()

    add_executable(slam_geo_anchor_tests
        tests/slam_geo_anchor_tests.cpp
        src/slam/SlamPoseUtils.h
        common/slam_types.h
        src/bridge/bridge_common_types.h
    )
    target_include_directories(slam_geo_anchor_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/common
        ${CMAKE_CURRENT_SOURCE_DIR}/src/slam
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge
        ${EIGEN3_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
    )
    target_link_libraries(slam_geo_anchor_tests PRIVATE
        Qt5::Core
        Eigen3::Eigen
    )
    add_test(NAME slam_geo_anchor COMMAND slam_geo_anchor_tests)

    add_executable(berth_recall_tests
        tests/berth_recall_tests.cpp
        src/bus/BerthPoseBuffer.h
        src/bus/BerthProtocolGeometry.h
        src/bus/RealtimeBerthRecall.h
        src/bus/RealtimeBerthCoordinateLibrary.h
        src/bus/common_types_extended.h
        src/bridge/bridge_common_types.h
        src/gui/BerthOverlayStyle.h
    )
    target_include_directories(berth_recall_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bus
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gui
        ${EIGEN3_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
    )
    target_link_libraries(berth_recall_tests PRIVATE
        Eigen3::Eigen
        ${OCV_IMPORT_LIBS}
    )
    add_test(NAME berth_recall COMMAND berth_recall_tests)

    add_executable(berth_occupancy_tests
        tests/berth_occupancy_tests.cpp
        src/bus/BerthOccupancy.h
        src/bus/common_types_extended.h
        src/bridge/bridge_common_types.h
    )
    target_include_directories(berth_occupancy_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bus
        ${EIGEN3_INCLUDE_DIR}
    )
    target_link_libraries(berth_occupancy_tests PRIVATE Eigen3::Eigen)
    add_test(NAME berth_occupancy COMMAND berth_occupancy_tests)

    add_executable(berth_world_stability_tests
        tests/berth_world_stability_tests.cpp
        src/bus/RealtimeBerthStability.h
        src/bus/RealtimeWorldAnchorBus.h
        src/bus/common_types_extended.h
    )
    target_include_directories(berth_world_stability_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bus
        ${EIGEN3_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
    )
    target_link_libraries(berth_world_stability_tests PRIVATE
        Eigen3::Eigen
    )
    add_test(NAME berth_world_stability COMMAND berth_world_stability_tests)

    add_executable(berth_opening_rules_tests
        tests/berth_opening_rules_tests.cpp
        src/berth/BerthOpeningRules.h
        src/bus/common_types_extended.h
    )
    target_include_directories(berth_opening_rules_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge
        ${CMAKE_CURRENT_SOURCE_DIR}/src/berth
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bus
        ${EIGEN3_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
    )
    target_link_libraries(berth_opening_rules_tests PRIVATE
        Eigen3::Eigen
    )
    add_test(NAME berth_opening_rules COMMAND berth_opening_rules_tests)

    add_executable(berth_line_geometry_tests
        tests/berth_line_geometry_tests.cpp
        src/bus/BerthLineGeometry.h
        src/berth/LineBerthDetection.h
        src/bus/BerthProtocolGeometry.h
        src/bus/common_types_extended.h
    )
    target_include_directories(berth_line_geometry_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge
        ${CMAKE_CURRENT_SOURCE_DIR}/src/berth
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bus
        ${EIGEN3_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
    )
    target_link_libraries(berth_line_geometry_tests PRIVATE
        Eigen3::Eigen
        ${OCV_IMPORT_LIBS}
    )
    add_test(NAME berth_line_geometry COMMAND berth_line_geometry_tests)

    add_executable(berth_rect_geometry_tests
        tests/berth_rect_geometry_tests.cpp
        src/berth/BerthRectGeometry.h
    )
    target_include_directories(berth_rect_geometry_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/berth
    )
    target_link_libraries(berth_rect_geometry_tests PRIVATE Eigen3::Eigen)
    add_test(NAME berth_rect_geometry COMMAND berth_rect_geometry_tests)

    add_executable(berth_recovery_rules_tests
        tests/berth_recovery_rules_tests.cpp
        src/berth/BerthRecoveryRules.h
        src/bus/common_types_extended.h
    )
    target_include_directories(berth_recovery_rules_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge
        ${CMAKE_CURRENT_SOURCE_DIR}/src/berth
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bus
        ${EIGEN3_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
    )
    target_link_libraries(berth_recovery_rules_tests PRIVATE
        Eigen3::Eigen
        ${OCV_IMPORT_LIBS}
    )
    add_test(NAME berth_recovery_rules COMMAND berth_recovery_rules_tests)

    add_executable(berth_udp_protocol_tests
        tests/berth_udp_protocol_tests.cpp
        src/bus/BerthUdpPacketBuilder.cpp
        src/bus/BerthUdpPacketBuilder.h
    )
    target_include_directories(berth_udp_protocol_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bus
        ${EIGEN3_INCLUDE_DIR}
    )
    target_link_libraries(berth_udp_protocol_tests PRIVATE
        Qt5::Core
        Eigen3::Eigen
    )
    add_test(NAME berth_udp_protocol COMMAND berth_udp_protocol_tests)

    add_executable(static_berth_library_tests
        tests/static_berth_library_tests.cpp
        src/bus/PerceptionUdpProtocol.h
        src/bus/StaticBerthLibrary.h
        src/bus/StaticBerthLibrary.cpp
        src/bus/BerthUdpPacketBuilder.h
        src/bus/BerthUdpPacketBuilder.cpp
    )
    target_include_directories(static_berth_library_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bus
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge
        ${EIGEN3_INCLUDE_DIR}
    )
    target_link_libraries(static_berth_library_tests PRIVATE
        Qt5::Core
        Eigen3::Eigen
    )
    add_test(NAME static_berth_library COMMAND static_berth_library_tests)

    add_executable(bridge_vision_update_tests
        tests/bridge_vision_update_tests.cpp
        src/core/AppConfig.cpp
        src/core/AppConfig.h
        src/core/CalibrationConfig.cpp
        src/core/CalibrationConfig.h
    )
    target_include_directories(bridge_vision_update_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bus
        ${CMAKE_CURRENT_SOURCE_DIR}/src/core
        ${EIGEN3_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
    )
    target_link_libraries(bridge_vision_update_tests PRIVATE
        Qt5::Core
        Qt5::Gui
        Eigen3::Eigen
        ${OCV_IMPORT_LIBS}
    )
    add_test(NAME bridge_vision_update COMMAND bridge_vision_update_tests)

    add_executable(pcap_playback_tests
        tests/pcap_playback_tests.cpp
        src/core/AppConfig.cpp
        src/core/AppConfig.h
        src/network/PcapPlaybackController.cpp
        src/network/PcapPlaybackController.h
        src/network/PcapPlaybackRequest.h
        src/network/PcapPlaybackTime.cpp
        src/network/PcapPlaybackTime.h
        src/network/PcapngReader.cpp
        src/network/PcapngReader.h
    )
    target_compile_definitions(pcap_playback_tests PRIVATE
        WPCAP
        HAVE_REMOTE
        _CRT_SECURE_NO_WARNINGS
    )
    target_include_directories(pcap_playback_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/core
        ${CMAKE_CURRENT_SOURCE_DIR}/src/network
        ${NPCAP_ROOT}/include
        ${NPCAP_ROOT}/include/pcap
    )
    target_link_libraries(pcap_playback_tests PRIVATE
        Qt5::Core
        Qt5::Network
        ${LOCAL_LIB_DIR}/libwpcap.a
        ${LOCAL_LIB_DIR}/libPacket.a
        ws2_32
    )
    add_test(NAME pcap_playback COMMAND pcap_playback_tests)

    add_executable(pause_snapshot_tests
        tests/pause_snapshot_tests.cpp
        src/debug/PauseSnapshot.cpp
        src/debug/PauseSnapshot.h
        src/network/PcapPlaybackTime.cpp
        src/network/PcapPlaybackTime.h
        common/pointxyz.h
    )
    target_include_directories(pause_snapshot_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/common
        ${CMAKE_CURRENT_SOURCE_DIR}/src/debug
        ${CMAKE_CURRENT_SOURCE_DIR}/src/network
    )
    target_link_libraries(pause_snapshot_tests PRIVATE
        Qt5::Core
        Qt5::Gui
    )
    add_test(NAME pause_snapshot COMMAND pause_snapshot_tests)

    add_executable(detection_control_tests
        tests/detection_control_tests.cpp
    )
    add_test(NAME detection_control COMMAND detection_control_tests)

    add_executable(detection_node_scheduler_tests
        tests/detection_node_scheduler_tests.cpp
        src/bus/DetectionNodeScheduler.h
    )
    target_include_directories(detection_node_scheduler_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bus
    )
    add_test(NAME detection_node_scheduler COMMAND detection_node_scheduler_tests)

    add_executable(detection_scheduler_config_tests
        tests/detection_scheduler_config_tests.cpp
        src/core/AppConfig.h
        src/core/AppConfig.cpp
    )
    target_include_directories(detection_scheduler_config_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/core
    )
    target_link_libraries(detection_scheduler_config_tests PRIVATE Qt5::Core)
    add_test(NAME detection_scheduler_config COMMAND detection_scheduler_config_tests)

    add_executable(perception_export_realtime_disabled_tests
        tests/perception_export_realtime_disabled_tests.cpp
    )
    set_target_properties(perception_export_realtime_disabled_tests PROPERTIES
        AUTOMOC OFF
        AUTOUIC OFF
        AUTORCC OFF)
    add_test(NAME perception_export_realtime_disabled
        COMMAND perception_export_realtime_disabled_tests)

    if(PCL_PERCEPTION_ENABLED)
        add_executable(pcl_cluster_preview_tests
            tests/pcl_cluster_preview_tests.cpp
            src/modules/pcl_cluster_preview.cpp
            src/modules/pcl_cluster_preview.h
        )
        target_include_directories(pcl_cluster_preview_tests PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/common
            ${CMAKE_CURRENT_SOURCE_DIR}/src/modules
            ${PCL_INCLUDE_DIRS}
        )
        target_compile_definitions(pcl_cluster_preview_tests PRIVATE
            PCL_SILENCE_MALLOC_WARNING=1)
        target_link_libraries(pcl_cluster_preview_tests PRIVATE
            Qt5::Core
            Qt5::Gui
            Eigen3::Eigen
            ${PCL_LIBRARIES}
        )
        add_test(NAME pcl_cluster_preview COMMAND pcl_cluster_preview_tests)
    endif()

endif()

if(BRIDGE_DETECT_ENABLED AND _NVCC_EXECUTABLE AND BRIDGE_CUDA_OBJ)
    # 勿对全目标定义 ENABLE_BRIDGE_DETECT，否则会破坏 Qt MOC；仅桥洞相关 .cpp 需要该宏
    message(STATUS "Bridge detect: ENABLE_BRIDGE_DETECT 将仅用于 bridge_detection_worker")

    # 手动 nvcc 编译 .cu；MinGW Kit 下通过 vcvars64 + cl 作为 host compiler
    if(_NVCC_VCVARS_BAT)
        set(_BRIDGE_CUDA_BAT "${CMAKE_BINARY_DIR}/compile_bridge_cuda.bat")
        file(TO_NATIVE_PATH "${_NVCC_VCVARS_BAT}" _NVCC_VCVARS_BAT_NATIVE)
        file(TO_NATIVE_PATH "${_NVCC_EXECUTABLE}" _NVCC_EXECUTABLE_NATIVE)
        file(TO_NATIVE_PATH "${_NVCC_HOST_CCBIN}" _NVCC_HOST_CCBIN_NATIVE)
        file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/bridge/bridge_cuda.cu" _BRIDGE_CUDA_CU_NATIVE)
        file(TO_NATIVE_PATH "${BRIDGE_CUDA_OBJ}" _BRIDGE_CUDA_OBJ_NATIVE)
        if(_NVCC_VCVARS_VER)
            set(_BRIDGE_VCVARS_LINE "call \"${_NVCC_VCVARS_BAT_NATIVE}\" -vcvars_ver=${_NVCC_VCVARS_VER} >nul 2>&1")
        else()
            set(_BRIDGE_VCVARS_LINE "call \"${_NVCC_VCVARS_BAT_NATIVE}\" >nul 2>&1")
        endif()
        file(WRITE "${_BRIDGE_CUDA_BAT}" "@echo off
${_BRIDGE_VCVARS_LINE}
if errorlevel 1 call \"${_NVCC_VCVARS_BAT_NATIVE}\" >nul 2>&1
if errorlevel 1 exit /b 1
\"${_NVCC_EXECUTABLE_NATIVE}\" -allow-unsupported-compiler -ccbin \"${_NVCC_HOST_CCBIN_NATIVE}\" -Xcompiler \"/D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH\" -Xcompiler \"/GS-\" -O2 -c \"${_BRIDGE_CUDA_CU_NATIVE}\" -o \"${_BRIDGE_CUDA_OBJ_NATIVE}\"
exit /b %ERRORLEVEL%
")
        add_custom_command(
            OUTPUT "${BRIDGE_CUDA_OBJ}"
            COMMAND "${_BRIDGE_CUDA_BAT}"
            DEPENDS
                "${CMAKE_CURRENT_SOURCE_DIR}/src/bridge/bridge_cuda.cu"
                "${_BRIDGE_CUDA_BAT}"
            COMMENT "Compiling bridge_cuda.cu with nvcc (CUDA + MSVC host)"
        )
    else()
        add_custom_command(
            OUTPUT "${BRIDGE_CUDA_OBJ}"
            COMMAND "${_NVCC_EXECUTABLE}"
                -allow-unsupported-compiler
                -ccbin "${_NVCC_HOST_CCBIN}"
                -Xcompiler "/D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH"
                -O2
                -c "${CMAKE_CURRENT_SOURCE_DIR}/src/bridge/bridge_cuda.cu"
                -o "${BRIDGE_CUDA_OBJ}"
            DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/bridge/bridge_cuda.cu"
            COMMENT "Compiling bridge_cuda.cu with nvcc"
            VERBATIM
        )
    endif()
    target_sources(${PROJECT_NAME} PRIVATE "${BRIDGE_CUDA_OBJ}")
    set_source_files_properties("${BRIDGE_CUDA_OBJ}" PROPERTIES GENERATED TRUE EXTERNAL_OBJECT TRUE)

    # 仅桥洞检测源文件需要 PCL 头；勿加到全目标（会破坏 Qt MOC 预处理）
    set_source_files_properties(
        "${CMAKE_CURRENT_SOURCE_DIR}/src/modules/bridge_detection_worker.cpp"
        PROPERTIES
            INCLUDE_DIRECTORIES "${PCL_INCLUDE_DIRS};${OPENCV_INCLUDE_DIR}"
            COMPILE_DEFINITIONS "ENABLE_BRIDGE_DETECT;PCL_SILENCE_MALLOC_WARNING=1")
    target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES})

    find_library(CUDA_CUDART_LIB NAMES cudart cudart.lib
        HINTS "${CUDAToolkit_ROOT}/lib/x64" "${CUDAToolkit_ROOT}/lib")
    if(CUDA_CUDART_LIB)
        target_link_libraries(${PROJECT_NAME} PRIVATE "${CUDA_CUDART_LIB}")
        message(STATUS "Bridge detect: linked ${CUDA_CUDART_LIB}")
    else()
        message(WARNING "Bridge detect: 未找到 cudart.lib，链接可能失败")
    endif()

    # 启动时必须能找到 cudart（已链接进 exe，缺 DLL 会一启动就崩溃）
    if(CUDAToolkit_ROOT AND EXISTS "${CUDAToolkit_ROOT}/bin/cudart64_110.dll")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CUDAToolkit_ROOT}/bin/cudart64_110.dll"
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
            COMMENT "Copying cudart64_110.dll next to executable..."
        )
    endif()

    if(EXISTS "${PCL_RUNTIME_DIR}")
        file(GLOB _PCL_RUNTIME_DLLS "${PCL_RUNTIME_DIR}/libpcl_*.dll")
        if(_PCL_RUNTIME_DLLS)
            add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${_PCL_RUNTIME_DLLS}
                    "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
                COMMENT "Copying libpcl_*.dll only (avoid overwriting ucrt64/Qt DLLs)..."
            )
        endif()
    endif()
endif()

if(SLAM_ENABLED)
    set(_slam_compile_srcs
        "${CMAKE_CURRENT_SOURCE_DIR}/src/slam/DloSlamNode.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/code-slam/LidarGnssSlam.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/code-slam/nano_gicp/nano_gicp.cc"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/code-slam/nano_gicp/lsq_registration.cc"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/code-slam/nano_gicp/nanoflann.cc"
    )
    set(_slam_headers_needing_pcl
        "${CMAKE_CURRENT_SOURCE_DIR}/src/slam/DloSlamNode.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/code-slam/LidarGnssSlam.h"
    )
    set_source_files_properties(${_slam_compile_srcs} ${_slam_headers_needing_pcl}
        PROPERTIES
            INCLUDE_DIRECTORIES "${PCL_INCLUDE_DIRS};${OPENCV_INCLUDE_DIR}"
            COMPILE_OPTIONS "-fopenmp")
    target_include_directories(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES})
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        ENABLE_SLAM=1
        PCL_SILENCE_MALLOC_WARNING=1
    )
    find_library(_slam_boost_format NAMES libboost_format.dll.a libboost_format.a
        PATHS "${_ucrt64_prefix}/lib" NO_DEFAULT_PATH)
    if(_slam_boost_format)
        target_link_libraries(${PROJECT_NAME} PRIVATE "${_slam_boost_format}")
    endif()
    find_library(_slam_gomp NAMES libgomp.dll.a libgomp.a gomp
        PATHS "${_ucrt64_prefix}/lib" NO_DEFAULT_PATH)
    if(_slam_gomp)
        target_link_libraries(${PROJECT_NAME} PRIVATE "${_slam_gomp}")
    endif()
    if(NOT BRIDGE_DETECT_ENABLED)
        foreach(_boost IN ITEMS boost_system boost_filesystem boost_iostreams boost_serialization)
            find_library(_slam_${_boost} NAMES "lib${_boost}.dll.a" "lib${_boost}.a"
                PATHS "${_ucrt64_prefix}/lib" NO_DEFAULT_PATH)
            if(_slam_${_boost})
                target_link_libraries(${PROJECT_NAME} PRIVATE "${_slam_${_boost}}")
            endif()
        endforeach()
        find_library(_slam_flann NAMES libflann.dll.a libflann.a flann
            PATHS "${_ucrt64_prefix}/lib" NO_DEFAULT_PATH)
        if(_slam_flann)
            target_link_libraries(${PROJECT_NAME} PRIVATE "${_slam_flann}")
        endif()
        if(EXISTS "${PCL_RUNTIME_DIR}")
            file(GLOB _PCL_SLAM_DLLS "${PCL_RUNTIME_DIR}/libpcl_*.dll")
            if(_PCL_SLAM_DLLS)
                add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        ${_PCL_SLAM_DLLS}
                        "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
                    COMMENT "Copying PCL DLLs for SLAM..."
                )
            endif()
        endif()
    endif()
endif()

# --- 10. 自动拷贝 DLL ---
if(WIN32)
    if(PCL_FOUND AND EXISTS "${PCL_RUNTIME_DIR}")
        file(GLOB _VCPKG_PCL_RUNTIME_DLLS "${PCL_RUNTIME_DIR}/*.dll")
        if(_VCPKG_PCL_RUNTIME_DLLS)
            add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${_VCPKG_PCL_RUNTIME_DLLS}
                    "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
                COMMENT "Copying vcpkg PCL runtime dependencies..."
            )
        endif()
    endif()

    set(_MINGW_RUNTIME_DLLS "")
    foreach(_runtime IN ITEMS
        libgcc_s_seh-1.dll
        libstdc++-6.dll
        libwinpthread-1.dll
        libgomp-1.dll)
        if(EXISTS "${_ucrt64_prefix}/bin/${_runtime}")
            list(APPEND _MINGW_RUNTIME_DLLS
                "${_ucrt64_prefix}/bin/${_runtime}")
        endif()
    endforeach()
    if(_MINGW_RUNTIME_DLLS)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${_MINGW_RUNTIME_DLLS}
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
            COMMENT "Copying MinGW UCRT runtime DLLs..."
        )
    endif()

    set(_QT_RUNTIME_DLLS "")
    foreach(_qt_runtime IN ITEMS
        Qt5Core.dll
        Qt5Gui.dll
        Qt5Widgets.dll
        Qt5Network.dll
        Qt5Concurrent.dll)
        if(EXISTS "${_ucrt64_prefix}/bin/${_qt_runtime}")
            list(APPEND _QT_RUNTIME_DLLS
                "${_ucrt64_prefix}/bin/${_qt_runtime}")
        endif()
    endforeach()
    if(_QT_RUNTIME_DLLS)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${_QT_RUNTIME_DLLS}
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
            COMMENT "Copying Qt runtime DLLs..."
        )
    endif()

    set(_QT_WINDOWS_PLATFORM_PLUGIN
        "${_ucrt64_prefix}/share/qt5/plugins/platforms/qwindows.dll")
    if(EXISTS "${_QT_WINDOWS_PLATFORM_PLUGIN}")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>/platforms"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_QT_WINDOWS_PLATFORM_PLUGIN}"
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>/platforms/qwindows.dll"
            COMMENT "Copying Qt Windows platform plugin..."
        )
    endif()

    set(_RUNTIME_DEPENDENCY_SEARCH_DIRS
        "${_ucrt64_prefix}/bin"
        "${PCL_RUNTIME_DIR}"
        "${FFMPEG_ROOT}/bin"
        "${LOCAL_LIB_DIR}")
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND}
            "-DAPP_EXECUTABLE=$<TARGET_FILE:${PROJECT_NAME}>"
            "-DDESTINATION=$<TARGET_FILE_DIR:${PROJECT_NAME}>"
            "-DSEARCH_DIRECTORIES=${_RUNTIME_DEPENDENCY_SEARCH_DIRS}"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CopyRuntimeDependencies.cmake"
        COMMENT "Resolving and copying recursive runtime DLL dependencies..."
        VERBATIM
    )

    set(_LS400_RUNTIME_DLL "${LOCAL_LIB_DIR}/libGetLidarData_LS.dll")
    if(EXISTS "${_LS400_RUNTIME_DLL}")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_LS400_RUNTIME_DLL}"
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
            COMMENT "Copying LS400 runtime DLL..."
        )
    endif()

    # copy_directory 在部分 Windows/杀毒软件环境下会因覆盖已有 DLL
    # 返回 Permission denied，进而使 make 删除刚生成的可执行文件。逐文件
    # copy_if_different 保留相同效果，同时避免整目录复制的失败语义。
    set(_FFMPEG_RUNTIME_FILES
        "${FFMPEG_ROOT}/bin/avcodec-62.dll"
        "${FFMPEG_ROOT}/bin/avdevice-62.dll"
        "${FFMPEG_ROOT}/bin/avfilter-11.dll"
        "${FFMPEG_ROOT}/bin/avformat-62.dll"
        "${FFMPEG_ROOT}/bin/avutil-60.dll"
        "${FFMPEG_ROOT}/bin/swresample-6.dll"
        "${FFMPEG_ROOT}/bin/swscale-9.dll"
        "${FFMPEG_ROOT}/bin/ffmpeg.exe"
        "${FFMPEG_ROOT}/bin/ffplay.exe"
        "${FFMPEG_ROOT}/bin/ffprobe.exe")
    foreach(_ffmpeg_runtime_file IN LISTS _FFMPEG_RUNTIME_FILES)
        if(EXISTS "${_ffmpeg_runtime_file}")
            add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${_ffmpeg_runtime_file}"
                    "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
                COMMENT "Copying FFmpeg runtime file..."
            )
        endif()
    endforeach()
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/config/config.json"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>/config.json"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/config/devices.json"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>/devices.json"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/config/calibration.json"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>/calibration.json"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/config/devices_bridge1.json"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>/devices_bridge1.json"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/config/devices_bridge2.json"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>/devices_bridge2.json"
        COMMENT "Copying config.json, device profiles and calibration.json..."
    )
endif()
