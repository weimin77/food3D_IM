#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#if !defined(_WIN32) && !defined(__ARM_EABI__)
#include <sys/resource.h>
#endif
#include "log.hpp"
#include "measurement.hpp"



namespace {

// Injected by test_package/CMakeLists.txt (link_to_resources); fallback for a standalone compile.
#ifndef RESOURCES_PATH
#define RESOURCES_PATH "."
#endif

const char* const kBaselinePcd = RESOURCES_PATH "/d405_260322274982_20260805_142739.pcd";
const char* const kFoodPcd = RESOURCES_PATH "/d405_260322274982_20260819_180010.pcd";

#ifndef __ARM_EABI__
using Clock = std::chrono::steady_clock;

// 墙钟时间（毫秒）。
double wall_ms(const Clock::time_point& a, const Clock::time_point& b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}

#if defined(_WIN32)
// Windows 不提供 getrusage：CPU 时间与峰值内存记为 -1（输出 N/A）。
struct CpuSnapshot {};
CpuSnapshot cpu_snapshot() { return CpuSnapshot{}; }
double cpu_ms_since(const CpuSnapshot& /*unused*/) { return -1.0; }
double peak_rss_mb() { return -1.0; }
#else
using CpuSnapshot = rusage;

CpuSnapshot cpu_snapshot() {
    rusage snapshot;
    getrusage(RUSAGE_SELF, &snapshot);
    return snapshot;
}

// 两次 rusage 之间的进程 CPU 时间（用户态 + 内核态，毫秒）。
double cpu_ms_since(const CpuSnapshot& before) {
    const rusage after = cpu_snapshot();
    const double user = (after.ru_utime.tv_sec - before.ru_utime.tv_sec) * 1000.0 +
                        (after.ru_utime.tv_usec - before.ru_utime.tv_usec) / 1000.0;
    const double system = (after.ru_stime.tv_sec - before.ru_stime.tv_sec) * 1000.0 +
                          (after.ru_stime.tv_usec - before.ru_stime.tv_usec) / 1000.0;
    return user + system;
}

// 进程峰值驻留内存（MB）。ru_maxrss 在 Linux 是 KB，在 macOS 是字节。
double peak_rss_mb() {
    rusage usage;
    getrusage(RUSAGE_SELF, &usage);
#if defined(__APPLE__)
    return static_cast<double>(usage.ru_maxrss) / (1024.0 * 1024.0);
#else
    return static_cast<double>(usage.ru_maxrss) / 1024.0;
#endif
}
#endif // _WIN32
#endif // __ARM_EABI__

} // namespace



int main(int argc, char* argv[]) {
#ifndef __ARM_EABI__
    // `main` 是独立 CLI 演示程序，不是 GTest 二进制；覆盖率构建路径会通过
    // gtest_discover_tests 以 `--gtest_list_tests` 探测它。识别到任意 gtest
    // 探测参数时直接干净退出（返回 0、不输出测试名）。
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]).rfind("--gtest", 0) == 0) {
            return 0;
        }
    }

    // 命令行参数：baseline PCD 与 food PCD 路径；未提供时退回内置默认值。
    const std::string baseline_path = (argc > 1) ? argv[1] : kBaselinePcd;
    const std::string food_path = (argc > 2) ? argv[2] : kFoodPcd;
    if (argc < 3) {
        std::cout << "用法: " << argv[0] << " <baseline.pcd> <food.pcd>" << std::endl;
        std::cout << "未提供参数，使用默认: " << baseline_path << " " << food_path << std::endl;
    }

    // 关闭日志（含文件写盘），避免 I/O 干扰 run() 计时；口径与 example/bench.cpp 一致。
    // 只保留 Error 级、且不走控制台；本程序自己负责输出结果与指标。
    log_set_level(LogLevel::kError);
    log_set_console(false);

    const PointCloud baseline = load_pcd(baseline_path);
    const PointCloud food = load_pcd(food_path);
    if (baseline.points.empty() || food.points.empty()) {
        std::cerr << "failed to load test point clouds" << std::endl;
        return 1;
    }
    std::cout << "\n=== 加载 PCD ===" << std::endl;
    std::cout << "baseline points: " << baseline.points.size() << std::endl;
    std::cout << "初始点云数量: " << food.points.size() << std::endl;

    FoodVolumeMeasurer measurer;
    measurer.set_baseline({baseline}).set_food(food);

    // 只对 run() 计时：点云加载与 setter 的拷贝不计入，与 example/bench.cpp 口径一致。
    const CpuSnapshot cpu_before = cpu_snapshot();
    const Clock::time_point wall_before = Clock::now();
    const VolumeEstimate est = measurer.run();
    const double run_wall_ms = wall_ms(wall_before, Clock::now());
    const double run_cpu_ms = cpu_ms_since(cpu_before);
    const double peak_rss = peak_rss_mb();

    std::cout << "\n=== IM 复现完成 ===" << std::endl;
    std::cout << "输入点数: " << est.input_points << std::endl;
    std::cout << "下采样点数: " << est.downsampled_points << std::endl;
    std::cout << "聚类数: " << est.cluster_count << std::endl;
    std::cout << "目标簇标签: [";
    for (std::size_t i = 0; i < est.selected_cluster_labels.size(); ++i) {
        if (i != 0) {
            std::cout << ", ";
        }
        std::cout << est.selected_cluster_labels[i];
    }
    std::cout << "]" << std::endl;
    std::cout << "目标连通块数: " << est.component_count << std::endl;
    std::cout << "目标簇点数: " << est.selected_cluster_points << std::endl;
    std::cout << "顶部 surface cell 数: " << est.top_surface_points << std::endl;
    std::cout << "baseline 帧数: " << est.baseline_frames << std::endl;
    std::cout << "占用网格数: " << est.occupied_cells << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Footprint 占用面积: " << est.footprint_area_m2 << " m^2" << std::endl;
    std::cout << "平均高度: " << est.mean_height_m << " m" << std::endl;
    std::cout << "最大高度: " << est.max_height_m << " m" << std::endl;
    std::cout << "PCD 原始积分体积: " << est.raw_volume_cm3 / 1e6 << " m^3" << std::endl;
    std::cout << "PCD 补洞后积分体积: " << est.volume_cm3 / 1e6 << " m^3" << std::endl;
    std::cout << "AABB 体积: " << est.aabb_volume_m3 << " m^3" << std::endl;
    std::cout << "OBB-Compact 体积: " << est.obb_volume_m3 << " m^3" << std::endl;
    std::cout << "Convex Hull 体积: " << est.convex_hull_volume_m3 << " m^3" << std::endl;
    std::cout << std::defaultfloat;
    std::cout << "未匹配 baseline cell 数: " << est.missing_baseline_cells << std::endl;

    // 性能指标：与 example/bench.cpp 相同的口径（仅 run() 计时，日志已关闭）。
    std::cout << "\n=== 性能指标 ===" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "run() 墙钟时间: " << run_wall_ms << " ms" << std::endl;
    if (run_cpu_ms >= 0.0) {
        std::cout << "run() CPU 时间: " << run_cpu_ms << " ms" << std::endl;
    } else {
        std::cout << "run() CPU 时间: N/A" << std::endl;
    }
    if (peak_rss >= 0.0) {
        std::cout << "峰值内存 max_rss: " << peak_rss << " MB" << std::endl;
    } else {
        std::cout << "峰值内存 max_rss: N/A" << std::endl;
    }
    std::cout << std::defaultfloat;

    return est.status == MeasurementStatus::kSuccess ? 0 : 1;
#else
    (void)argc;
    (void)argv;
    std::cout << "volume measurement unavailable on bare-metal" << std::endl;
    return 0;
#endif
}
