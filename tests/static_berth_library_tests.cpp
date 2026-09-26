#include "PerceptionUdpProtocol.h"
#include "StaticBerthLibrary.h"

#include <QFile>
#include <QTemporaryFile>

#include <iostream>
#include <string>
#include <vector>

namespace {

void check(bool condition, const char *message, int &failures)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        ++failures;
    }
}

}  // namespace

int main(int argc, char **argv)
{
    if (argc > 1) {
        static_berth_library::LoadResult result;
        QString error;
        if (!static_berth_library::load(
                QString::fromLocal8Bit(argv[1]), result, &error)) {
            std::cerr << "FAIL: " << error.toStdString() << "\n";
            return 1;
        }
        std::cout << "Loaded static berth count: " << result.units.size()
                  << "\n";
        return result.units.empty() ? 1 : 0;
    }

    QTemporaryFile fixture;
    fixture.setAutoRemove(true);
    if (!fixture.open()) {
        std::cerr << "FAIL: could not create JSON fixture\n";
        return 1;
    }

    const QByteArray json = R"json({
      "frames": [{"detections": [
        {"record_class":"berth_library","is_final_output":true,
         "berth_type":{"code":2},
         "center":{"east_m":10.0,"north_m":20.0},
         "vertices":[
           {"latitude_deg":35.0,"longitude_deg":119.0,"east_m":8.0,"north_m":18.0},
           {"latitude_deg":35.0,"longitude_deg":119.0,"east_m":12.0,"north_m":18.0},
           {"latitude_deg":35.0,"longitude_deg":119.0,"east_m":12.0,"north_m":22.0},
           {"latitude_deg":35.0,"longitude_deg":119.0,"east_m":8.0,"north_m":22.0}
         ],
         "width_m":4.0,"length_m":4.0,"angle_deg":0.0},
        {"record_class":"berth_library","is_final_output":true,
         "berth_type":{"code":2},
         "center":{"east_m":10.2,"north_m":20.1},
         "vertices":[
           {"latitude_deg":35.0,"longitude_deg":119.0,"east_m":8.2,"north_m":18.1},
           {"latitude_deg":35.0,"longitude_deg":119.0,"east_m":12.2,"north_m":18.1},
           {"latitude_deg":35.0,"longitude_deg":119.0,"east_m":12.2,"north_m":22.1},
           {"latitude_deg":35.0,"longitude_deg":119.0,"east_m":8.2,"north_m":22.1}
         ],
         "width_m":4.0,"length_m":4.0,"angle_deg":0.0},
        {"record_class":"berth_library","is_final_output":true,
         "berth_type":{"code":1},
         "center":{"east_m":30.0,"north_m":20.0},
         "vertices":[
           {"latitude_deg":35.0,"longitude_deg":119.0,"east_m":28.0,"north_m":18.0},
           {"latitude_deg":35.0,"longitude_deg":119.0,"east_m":32.0,"north_m":18.0},
           {"latitude_deg":35.0,"longitude_deg":119.0,"east_m":32.0,"north_m":22.0},
           {"latitude_deg":35.0,"longitude_deg":119.0,"east_m":28.0,"north_m":22.0}
         ],
         "width_m":4.0,"length_m":4.0,"angle_deg":0.0}
      ]}]
    })json";
    fixture.write(json);
    fixture.flush();

    static_berth_library::LoadResult result;
    QString error;
    const bool loaded = static_berth_library::load(
        fixture.fileName(), result, &error);
    int failures = 0;
    check(loaded, "static berth JSON should load", failures);
    check(error.isEmpty(), "successful load should not report an error", failures);
    check(result.units.size() == 2,
          "nearby repeated records should be deduplicated", failures);
    check(result.display_berths.size() == 2,
          "固定泊位解析结果必须同时提供本地绘制记录", failures);
    if (result.display_berths.size() == 2) {
        check(result.display_berths[0].cx == 10.0
                  && result.display_berths[0].cy == 20.0,
              "本地绘制泊位必须保留 ENU 中心", failures);
    }
    if (result.units.size() == 2) {
        check(result.units[0].type == 2,
              "first static berth should preserve protocol type", failures);
        check(result.units[1].type == 1,
              "second static berth should preserve protocol type", failures);
        check(result.units[0].x1 == 80 && result.units[0].y1 == 180,
              "static berth should preserve ENU corner coordinates", failures);
    }

    if (failures == 0)
        std::cout << "Static berth library tests passed\n";
    return failures == 0 ? 0 : 1;
}
