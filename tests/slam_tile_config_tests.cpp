#include "AppConfig.h"

#include <QCoreApplication>
#include <QFile>
#include <QTemporaryDir>

#include <cmath>
#include <iostream>

namespace {

void check(bool condition, const char *message, int &failures)
{
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

}  // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    int failures = 0;
    QTemporaryDir directory;
    check(directory.isValid(), "temporary config directory should exist",
          failures);
    if (!directory.isValid())
        return 1;

    const QString path = directory.filePath(QStringLiteral("config.json"));
    QFile file(path);
    check(file.open(QIODevice::WriteOnly | QIODevice::Truncate),
          "tile config fixture should open", failures);
    file.write(R"JSON({
      "slam": {
        "offline_tile_map": {
          "enabled": true,
          "tile_size_m": 42.0,
          "lod_voxel_sizes_m": [2.0, 0.5, 0.2],
          "prefetch_ring_tiles": 2,
          "unload_ring_tiles": 1,
          "ram_budget_mb": 700,
          "gpu_point_budget": 1234567,
          "view_update_delay_ms": 75,
          "max_concurrent_loads": 3
        }
      }
    })JSON");
    file.close();

    check(AppConfig::instance().load(path), "tile config should load", failures);
    const SlamTileMapAppConfig &config =
        AppConfig::instance().slam().offline_tile_map;
    check(config.enabled && std::abs(config.tile_size_m - 42.0) < 1e-9,
          "tile enable flag and size should load", failures);
    check(config.lod_voxel_sizes_m.size() == 3
              && std::abs(config.lod_voxel_sizes_m[0] - 2.0) < 1e-9
              && std::abs(config.lod_voxel_sizes_m[2] - 0.2) < 1e-9,
          "all configured LOD sizes should load", failures);
    check(config.prefetch_ring_tiles == 2
              && config.unload_ring_tiles >= config.prefetch_ring_tiles,
          "unload ring should never be smaller than prefetch", failures);
    check(config.ram_budget_mb == 700
              && config.gpu_point_budget == 1234567
              && config.view_update_delay_ms == 75
              && config.max_concurrent_loads == 3,
          "tile runtime budgets should load", failures);

    if (failures != 0)
        return 1;
    std::cout << "All SLAM tile config tests passed\n";
    return 0;
}
