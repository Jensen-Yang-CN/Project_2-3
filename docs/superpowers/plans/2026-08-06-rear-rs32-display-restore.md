# 213/214 RS32 尾部雷达显示恢复实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 让 `213/214` 两路 RS32 点云恢复到主界面显示，同时确保 SLAM、泊位检测和桥梁检测仍只使用 `210/211`。

**架构：** 将原来的单一融合名单拆成显示名单和算法名单，并让 `RadarFusionManager` 在同一个 `210` 主时钟同步周期内生成显示点云和算法点云两个输出。主界面只渲染四雷达显示输出，总线只接收双雷达算法输出；`201` 不进入任何输出。

**技术栈：** C++17、Qt5 Core/Gui/Widgets、Qt signals/slots、CMake/CTest、现有 `RadarTransform` 与 `CalibrationConfig`。

---

## 文件结构

- 修改 `src/modules/RadarFusionPolicy.h`：集中定义显示名单、算法名单和成员判断。
- 修改 `tests/radar_update_tests.cpp`：用 TDD 固定两套名单及 `201` 排除规则。
- 创建 `tests/radar_fusion_output_tests.cpp`：真实驱动 `RadarFusionManager` 定时融合，验证两份输出的数据隔离、同步容差和缺帧降级。
- 修改 `src/modules/radarfusionmanager.h`：将单一输出信号拆成显示输出与算法输出。
- 修改 `src/modules/radarfusionmanager.cpp`：一次同步选择和坐标变换，同时构造两份点云。
- 修改 `src/gui/mainwindow.cpp`：显示信号只连接点云控件，算法信号只连接融合总线入口。
- 修改 `CMakeLists.txt`：注册新的融合输出回归测试目标。

> 当前目录不是 Git 工作区，因此计划中的每个任务以测试检查点收尾，不包含无法执行的 commit 步骤。

### 任务 1：拆分显示与算法雷达策略

**文件：**
- 修改：`tests/radar_update_tests.cpp:28-39`
- 修改：`src/modules/RadarFusionPolicy.h:8-25`

- [ ] **步骤 1：先把现有测试改成期望的新策略**

用以下测试替换 `testOnlyFrontLidarsEnterFusedCloud()`：

```cpp
void testDisplayAndAlgorithmRadarMembership()
{
    const QStringList displayIds =
        radar_fusion_policy::displayRadarIds();
    const QStringList algorithmIds =
        radar_fusion_policy::algorithmRadarIds();

    check(displayIds == QStringList({"210", "211", "213", "214"}),
          "显示点云必须包含 210/211/213/214");
    check(algorithmIds == QStringList({"210", "211"}),
          "算法点云必须继续只包含 210/211");
    check(!displayIds.contains(QStringLiteral("201"))
              && !algorithmIds.contains(QStringLiteral("201")),
          "201 不得进入显示或算法输出");
    check(!radar_fusion_policy::isAlgorithmRadar("213")
              && !radar_fusion_policy::isAlgorithmRadar("214"),
          "213/214 不得进入 SLAM 或泊位检测使用的算法点云");
}
```

同步修改 `main()` 中的调用：

```cpp
testDisplayAndAlgorithmRadarMembership();
```

- [ ] **步骤 2：运行测试，确认因新接口不存在而失败**

运行：

```powershell
$env:PATH = 'D:\msys64\ucrt64\bin;' + $env:PATH
cmake --build build-radar-slam --target radar_update_tests -j 4
```

预期：编译失败，错误指出 `displayRadarIds`、`algorithmRadarIds` 或 `isAlgorithmRadar` 尚未定义。该失败证明测试覆盖了本次策略变化。

- [ ] **步骤 3：实现最小策略接口**

将 `RadarFusionPolicy.h` 的单一 `activeRadarIds()` 替换为：

```cpp
inline const QStringList &displayRadarIds()
{
    static const QStringList ids = {
        QStringLiteral("210"), QStringLiteral("211"),
        QStringLiteral("213"), QStringLiteral("214")};
    return ids;
}

inline const QStringList &algorithmRadarIds()
{
    static const QStringList ids = {
        QStringLiteral("210"), QStringLiteral("211")};
    return ids;
}

inline bool isAlgorithmRadar(const QString &id)
{
    return algorithmRadarIds().contains(id);
}
```

更新头文件注释，明确 `213/214` 只恢复显示，`201` 继续排除。

- [ ] **步骤 4：运行策略测试确认变绿**

运行：

```powershell
$env:PATH = 'D:\msys64\ucrt64\bin;' + $env:PATH
cmake --build build-radar-slam --target radar_update_tests -j 4
if ($LASTEXITCODE -eq 0) { .\build-radar-slam\radar_update_tests.exe }
```

预期：输出 `All radar update tests passed`。

### 任务 2：用真实 RadarFusionManager 测试锁定双输出行为

**文件：**
- 创建：`tests/radar_fusion_output_tests.cpp`
- 修改：`CMakeLists.txt:617-636` 附近的测试目标区

- [ ] **步骤 1：创建失败的融合输出测试**

创建 `tests/radar_fusion_output_tests.cpp`，使用实际 Qt 定时器和信号：

```cpp
#include "radarfusionmanager.h"

#include <QCoreApplication>
#include <QEventLoop>
#include <QSet>
#include <QTimer>

#include <iostream>

namespace {
int failures = 0;

void check(bool condition, const char *message)
{
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

QVector<M_PointXYZI> onePoint(float x)
{
    return {M_PointXYZI{x, 0.0F, 0.0F, 1, {0, 0, 0}}};
}

void waitForFusion()
{
    QEventLoop loop;
    QTimer::singleShot(40, &loop, &QEventLoop::quit);
    loop.exec();
}

QSet<int> intensities(const QVector<M_PointXYZI> &cloud)
{
    QSet<int> result;
    for (const M_PointXYZI &point : cloud)
        result.insert(point.intensity);
    return result;
}

void testRearRs32OnlyEntersDisplayOutput()
{
    RadarFusionManager manager;
    QVector<M_PointXYZI> display;
    QVector<M_PointXYZI> algorithm;
    QObject::connect(&manager, &RadarFusionManager::displayCloudReady,
                     [&](const QVector<M_PointXYZI> &cloud, double) {
        display = cloud;
    });
    QObject::connect(&manager, &RadarFusionManager::algorithmCloudReady,
                     [&](const QVector<M_PointXYZI> &cloud, double) {
        algorithm = cloud;
    });

    manager.handleRadarCloud("201", onePoint(1.0F), 100.0);
    manager.handleRadarCloud("211", onePoint(2.0F), 100.0);
    manager.handleRadarCloud("213", onePoint(3.0F), 100.0);
    manager.handleRadarCloud("214", onePoint(4.0F), 100.0);
    manager.handleRadarCloud("210", onePoint(5.0F), 100.0);
    waitForFusion();

    check(display.size() == 4,
          "显示输出必须包含 210/211/213/214 四路点云");
    check(algorithm.size() == 2,
          "算法输出必须只包含 210/211 两路点云");
    check(intensities(display) == QSet<int>({100, 150, 200, 250}),
          "显示输出必须包含四路雷达的显示强度且排除 201");
    check(intensities(algorithm) == QSet<int>({150, 250}),
          "算法输出不得包含 213/214/201");
}

void testStaleRearFramesAreSkippedWithoutBlockingFrontLidars()
{
    RadarFusionManager manager;
    QVector<M_PointXYZI> display;
    QVector<M_PointXYZI> algorithm;
    QObject::connect(&manager, &RadarFusionManager::displayCloudReady,
                     [&](const QVector<M_PointXYZI> &cloud, double) {
        display = cloud;
    });
    QObject::connect(&manager, &RadarFusionManager::algorithmCloudReady,
                     [&](const QVector<M_PointXYZI> &cloud, double) {
        algorithm = cloud;
    });

    manager.handleRadarCloud("213", onePoint(3.0F), 99.8);
    manager.handleRadarCloud("214", onePoint(4.0F), 99.8);
    manager.handleRadarCloud("211", onePoint(2.0F), 100.0);
    manager.handleRadarCloud("210", onePoint(5.0F), 100.0);
    waitForFusion();

    check(display.size() == 2 && algorithm.size() == 2,
          "过期 RS32 帧必须被跳过且不能阻塞 210/211");
}
} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    testRearRs32OnlyEntersDisplayOutput();
    testStaleRearFramesAreSkippedWithoutBlockingFrontLidars();
    if (failures != 0) {
        std::cerr << failures << " radar fusion output assertion(s) failed\n";
        return 1;
    }
    std::cout << "All radar fusion output tests passed\n";
    return 0;
}
```

- [ ] **步骤 2：在 CMake 中注册测试目标**

在 `BUILD_TESTING` 区域加入：

```cmake
add_executable(radar_fusion_output_tests
    tests/radar_fusion_output_tests.cpp
    src/modules/radarfusionmanager.cpp
    src/modules/radarfusionmanager.h
    src/modules/RadarFusionPolicy.h
    src/modules/radartransform.h
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
```

- [ ] **步骤 3：运行测试确认正确失败**

运行：

```powershell
$env:PATH = 'D:\msys64\ucrt64\bin;' + $env:PATH
cmake --build build-radar-slam --target radar_fusion_output_tests -j 4
```

预期：编译失败，错误指出 `displayCloudReady` 和 `algorithmCloudReady` 尚不存在。若测试因其他原因失败，先修正测试工程，直到失败原因只剩缺失的新信号。

### 任务 3：实现 RadarFusionManager 双输出

**文件：**
- 修改：`src/modules/radarfusionmanager.h:25-31`
- 修改：`src/modules/radarfusionmanager.cpp:58-118`

- [ ] **步骤 1：把单一信号替换为两个职责信号**

在 `radarfusionmanager.h` 中替换 `fusedCloudReady`：

```cpp
signals:
    void displayCloudReady(const QVector<M_PointXYZI> &displayCloud,
                           double timestampSec);
    void algorithmCloudReady(const QVector<M_PointXYZI> &algorithmCloud,
                             double timestampSec);
```

- [ ] **步骤 2：一次遍历构造两份输出**

将 `fuseSynchronized()` 的结果容器和雷达名单改为：

```cpp
QVector<M_PointXYZI> displayCloud;
QVector<M_PointXYZI> algorithmCloud;
const QStringList &radarIds = radar_fusion_policy::displayRadarIds();
```

在每个同步有效雷达的点循环中，完成现有坐标变换后加入两套容器：

```cpp
displayCloud.append(tp);
if (radar_fusion_policy::isAlgorithmRadar(id))
    algorithmCloud.append(tp);
```

函数尾部改成分别发送：

```cpp
if (!displayCloud.isEmpty())
    emit displayCloudReady(displayCloud, masterTs);
if (!algorithmCloud.isEmpty())
    emit algorithmCloudReady(algorithmCloud, masterTs);
```

不得降低或绕过现有 `isFrameSynchronized()` 的 `0.10` 秒容差。

- [ ] **步骤 3：运行融合输出测试确认通过**

运行：

```powershell
$env:PATH = 'D:\msys64\ucrt64\bin;' + $env:PATH
cmake --build build-radar-slam --target radar_fusion_output_tests -j 4
if ($LASTEXITCODE -eq 0) { .\build-radar-slam\radar_fusion_output_tests.exe }
```

预期：输出 `All radar fusion output tests passed`。

- [ ] **步骤 4：重新运行策略测试防止名单回退**

运行：

```powershell
.\build-radar-slam\radar_update_tests.exe
```

预期：输出 `All radar update tests passed`。

### 任务 4：把显示和算法信号接到正确消费者

**文件：**
- 修改：`src/gui/mainwindow.cpp:792-797`

- [ ] **步骤 1：修改两条信号连接**

将现有两条 `fusedCloudReady` 连接替换为：

```cpp
connect(radarFusionManager, &RadarFusionManager::displayCloudReady,
        ui->openGLWidget, &MultiLidarWidget::updateFusedCloud,
        Qt::QueuedConnection);
connect(radarFusionManager, &RadarFusionManager::algorithmCloudReady,
        this, &MainWindow::onFusedCloudToBus,
        Qt::QueuedConnection);
```

不要把 `displayCloudReady` 连接到 `onFusedCloudToBus()`。

- [ ] **步骤 2：静态检查所有旧信号调用点已迁移**

运行：

```powershell
rg -n "fusedCloudReady|displayCloudReady|algorithmCloudReady" src tests
```

预期：

- `src/modules/radarfusionmanager.h/.cpp` 只出现两个新信号；
- `mainwindow.cpp` 中显示信号只连接 `updateFusedCloud`；
- `mainwindow.cpp` 中算法信号只连接 `onFusedCloudToBus`；
- 不再存在 `RadarFusionManager::fusedCloudReady`。

- [ ] **步骤 3：构建主程序验证 Qt MOC 和信号签名**

运行：

```powershell
$env:PATH = 'D:\msys64\ucrt64\bin;F:\c\vcpkg\installed\x64-mingw-dynamic-bigobj\debug\bin;F:\c\vcpkg\installed\x64-mingw-dynamic-bigobj\bin;' + $env:PATH
cmake --build build-radar-slam --target PointCloud -j 4
```

预期：`PointCloud` 构建退出码为 `0`，生成 `build-radar-slam/PointCloud.exe`。

### 任务 5：完整验证与交付检查

**文件：**
- 验证：`build-radar-slam/PointCloud.exe`
- 验证：全部 CTest 目标

- [ ] **步骤 1：重新构建两个相关回归目标**

运行：

```powershell
$env:PATH = 'D:\msys64\ucrt64\bin;F:\c\vcpkg\installed\x64-mingw-dynamic-bigobj\debug\bin;F:\c\vcpkg\installed\x64-mingw-dynamic-bigobj\bin;' + $env:PATH
cmake --build build-radar-slam --target radar_update_tests -j 4
cmake --build build-radar-slam --target radar_fusion_output_tests -j 4
```

预期：两个目标都构建成功。

- [ ] **步骤 2：运行全部自动化测试**

运行：

```powershell
ctest --test-dir build-radar-slam --output-on-failure
```

预期：原有 `13` 项测试加新 `radar_fusion_output` 测试，共 `14/14` 通过、`0` 失败。

- [ ] **步骤 3：逐项核对数据隔离**

运行：

```powershell
rg -n "displayRadarIds|algorithmRadarIds|isAlgorithmRadar" src/modules tests
rg -n "displayCloudReady|algorithmCloudReady|onFusedCloudToBus|updateFusedCloud" src/gui/mainwindow.cpp src/modules
```

验收：显示路径包含 `213/214`，算法路径只包含 `210/211`，`201` 不在任一名单。

- [ ] **步骤 4：真实 PCAP 人工验收说明**

若本机可取得老师所用 PCAP：加载文件并保持 SLAM/检测开关状态不变，观察主点云界面出现尾部两路点云；再查看日志确认泊位检测和 SLAM 输入仍来自双雷达算法输出。若当前工作区没有该 PCAP，最终报告必须明确“自动化隔离已验证，真实 PCAP 视觉验收待用户运行”，不得把未执行的人工验收声明为通过。
