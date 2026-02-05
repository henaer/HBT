ESP32 智能自行车尾灯 🚴💡

<p align="center">

<img src="docs/images/banner.png" alt="ESP32智能自行车尾灯" width="600">

</p>

<p align="center">

基于ESP32的智能、安全增强型自行车尾灯系统

</p>

<p align="center">

<a href="#特性亮点">特性亮点</a> •

<a href="#硬件清单">硬件清单</a> •

<a href="#软件架构">软件架构</a> •

<a href="#快速开始">快速开始</a> •

<a href="#文档">文档</a> •

<a href="#贡献指南">贡献指南</a> •

<a href="#许可证">许可证</a>

</p>

<p align="center">

<a href="https://github.com/(https://github.com/)你的用户名/esp32-bike-tail-light/stargazers">

<img src="https://img.shields.io/github/stars/(https://img.shields.io/github/stars/)你的用户名/esp32-bike-tail-light?style=for-the-badge" alt="GitHub stars">

</a>

<a href="https://github.com/(https://github.com/)你的用户名/esp32-bike-tail-light/blob/main/LICENSE">

<img src="https://img.shields.io/github/license/(https://img.shields.io/github/license/)你的用户名/esp32-bike-tail-light?style=for-the-badge" alt="许可证">

</a>

<a href="https://github.com/(https://github.com/)你的用户名/esp32-bike-tail-light/issues">

<img src="https://img.shields.io/github/issues/(https://img.shields.io/github/issues/)你的用户名/esp32-bike-tail-light?style=for-the-badge" alt="GitHub issues">

</a>

<a href="https://github.com/(https://github.com/)你的用户名/esp32-bike-tail-light/commits/main">

<img src="https://img.shields.io/github/last-commit/(https://img.shields.io/github/last-commit/)你的用户名/esp32-bike-tail-light?style=for-the-badge" alt="GitHub 最后提交">

</a>

</p>

📋 目录

• 项目概述(#项目概述)

• ✨ 特性亮点(#特性亮点)

• 🛠️ 硬件清单(#硬件清单)

• 🔌 引脚配置(#引脚配置)

• 💻 软件架构(#软件架构)

• 🚀 快速开始(#快速开始)

• 📖 使用指南(#使用指南)

• 🧪 测试与验证(#测试与验证)

• 🐛 故障排除(#故障排除)

• 📊 性能规格(#性能规格)

• 📁 项目结构(#项目结构)

• 🤝 贡献指南(#贡献指南)

• 📄 许可证(#许可证)

• 📞 支持与联系(#支持与联系)

项目概述

ESP32 智能自行车尾灯 是一个开源的、功能丰富的自行车安全照明系统。围绕强大的ESP32微控制器构建，它结合了智能灯光模式、实时安全监控和连接功能，显著提高了夜间骑行或低光照条件下的骑手可见性和安全性。

核心设计原则：

• 安全第一：多重冗余安全功能

• 能效优化：优化电源管理，延长电池寿命

• 模块化设计：易于扩展的硬件和软件架构

• 用户友好：直观的控制和最小的学习曲线

✨ 特性亮点

🚦 智能照明系统

• 7种预设模式：常亮、呼吸、闪烁、彩虹、跑马、反应和自动安全模式

• 自适应亮度：根据环境光条件自动调整

• 刹车检测：MPU6050加速度计检测减速并触发紧急照明

• 动态效果：平滑过渡和引人注目的灯光模式

🛡️ 安全与监控

• 障碍物检测：超声波传感器警告接近车辆（20-200cm范围）

• 碰撞检测：检测到冲击时自动激活紧急模式

• 防盗功能：基于GPS的地理围栏和运动警报（可选模块）

• 低电量警报：电池电量低于20%时发出视觉和声音警告

🔋 电源管理

• 高效设计：深度睡眠模式将空闲功耗降至<5mA

• 智能充电：基于TP4056的充电，带温度监控

• 电池保护：过充、过放和短路保护

• 运行时间优化：基于使用模式的自适应功率调节

📡 连接功能

• 蓝牙LE：通过智能手机应用进行无线控制

• WiFi功能：OTA更新和远程配置

• GPS集成：骑行跟踪和防盗恢复（NEO-6M模块）

• 数据记录：SD卡支持骑行统计（可选）

🎛️ 用户界面

• 触觉控制：三按钮界面，用于模式选择和亮度控制

• 声音反馈：蜂鸣器提供确认音和警告

• 状态指示：多色LED显示电池、连接和错误状态

• 移动应用：Android/iOS配套应用进行高级控制

🛠️ 硬件清单

基本组件

组件	规格	数量	预估成本	备注
ESP32开发板	ESP32-WROOM-32	1	¥25-35	双核240MHz，WiFi/BLE
WS2812B灯带	60灯/米，IP67	16颗	¥15-20	防水，可单独寻址
MPU6050模块	6轴陀螺仪+加速度计	1	¥8-12	I2C接口，用于刹车检测
HC-SR04模块	超声波传感器	1	¥6-8	检测后方障碍物
18650电池	2600mAh，带保护板	1	¥15-20	锂电池供电
TP4056模块	充电管理模块	1	¥3-5	带USB Type-C接口
升压模块	3.7V转5V，2A	1	¥5-8	为LED供电
按钮开关	6×6×5mm 触觉开关	3	¥1-2	模式/亮度/电源控制
光敏电阻	GL5516型	1	¥2-3	环境光检测
蜂鸣器	有源5V蜂鸣器	1	¥2-3	声音反馈
合计	-	-	约¥80-120	基本版本

可选组件

组件	用途	参考型号
GPS模块	位置跟踪与防盗	NEO-6M
OLED显示屏	状态信息显示	SSD1306 0.96英寸
温湿度传感器	环境监测	DHT22
SD卡模块	数据记录	Micro SD卡模块
蓝牙模块	增强连接（如果ESP32 BLE不够）	HC-05/HC-06

🔌 引脚配置

ESP32 GPIO 分配

GPIO 引脚	连接组件	功能说明
GPIO4	WS2812B 数据线	LED灯带控制
GPIO21	MPU6050 SDA	I2C数据线
GPIO22	MPU6050 SCL	I2C时钟线
GPIO12	HC-SR04 Trig	超声波触发
GPIO13	HC-SR04 Echo	超声波回波
GPIO15	模式按钮	短按切换模式，长按校准
GPIO32	亮度按钮	亮度调节
GPIO33	电源按钮	开关机控制
GPIO34	光敏电阻	ADC输入，环境光检测
GPIO14	蜂鸣器	声音输出
GPIO27	充电检测	检测充电状态
GPIO35	电池电压检测	ADC输入，电池监控

电源连接图

18650电池 (3.7V)
    ├── TP4056充电模块 (USB Type-C输入)
    ├── 升压模块 (3.7V→5V, 2A输出)
    │   ├── ESP32 (5V Vin)
    │   ├── WS2812B灯带 (5V)
    │   └── 传感器模块 (5V)
    └── 分压电路 (ESP32 ADC)


💻 软件架构

系统架构图

┌─────────────────────────────────────────────┐
│                应用层                        │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐       │
│  │灯光效果 │ │用户界面 │ │安全监控 │       │
│  │管理器   │ │控制器   │ │系统     │       │
│  └─────────┘ └─────────┘ └─────────┘       │
└───────────────────┬─────────────────────────┘
                    │
┌─────────────────────────────────────────────┐
│                中间件层                      │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐       │
│  │传感器   │ │电源管理 │ │通信协议 │       │
│  │驱动     │ │模块     │ │栈       │       │
│  └─────────┘ └─────────┘ └─────────┘       │
└───────────────────┬─────────────────────────┘
                    │
┌─────────────────────────────────────────────┐
│                硬件抽象层                    │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐       │
│  │GPIO控制 │ │ADC/DAC  │ │I2C/SPI  │       │
│  │接口     │ │接口     │ │接口     │       │
│  └─────────┘ └─────────┘ └─────────┘       │
└─────────────────────────────────────────────┘


核心模块

1. 灯光效果引擎 (effects.cpp/h)

// 支持的效果模式
enum LightMode {
    MODE_STEADY,     // 常亮模式
    MODE_BREATHING,  // 呼吸效果
    MODE_FLASHING,   // 闪烁模式
    MODE_RAINBOW,    // 彩虹效果
    MODE_RUNNING,    // 跑马灯
    MODE_REACTIVE,   // 反应模式
    MODE_SAFETY,     // 自动安全模式
    MODE_COUNT       // 模式总数
};

class LightEffectManager {
public:
    void setMode(LightMode mode);
    void setBrightness(uint8_t level);
    void update();
    
private:
    // 各种效果的实现方法
    void renderSteadyMode();
    void renderBreathingMode();
    void renderRainbowMode();
    // ...
};


2. 传感器融合系统 (sensor_fusion.cpp/h)

class SensorFusion {
public:
    void init();
    void update();
    
    // 获取处理后的数据
    float getAcceleration();
    float getBrakeIntensity();
    float getObstacleDistance();
    bool isBraking() const;
    
private:
    MPU6050 mpu;
    HC_SR04 ultrasonic;
    
    // 滤波器
    KalmanFilter accelFilter;
    MovingAverage distanceFilter;
    
    // 状态机
    BrakeDetectionState brakeState;
};


3. 电源管理系统 (power_manager.cpp/h)

class PowerManager {
public:
    enum PowerState {
        STATE_ACTIVE,
        STATE_IDLE,
        STATE_SLEEP,
        STATE_DEEP_SLEEP
    };
    
    void monitorBattery();
    void enterSleepMode();
    void wakeUp();
    
    float getBatteryVoltage() const;
    int getBatteryPercentage() const;
    PowerState getCurrentState() const;
    
private:
    void updatePowerState();
    void handleLowBattery();
};


🚀 快速开始

环境准备

选项1：使用PlatformIO（推荐）

# 1. 安装PlatformIO Core
pip install platformio

# 2. 克隆本仓库
git clone https://github.com/你的用户名/esp32-bike-tail-light.git
cd esp32-bike-tail-light

# 3. 安装依赖库
pio pkg install

# 4. 编译项目
pio run

# 5. 上传到ESP32
pio run --target upload

# 6. 监视串口输出
pio run --target monitor


选项2：使用Arduino IDE

1. 安装Arduino IDE（版本1.8.19或更高）

2. 添加ESP32开发板支持：

  ◦ 文件 → 首选项 → 附加开发板管理器网址

  ◦ 添加：https://espressif.github.io/arduino-esp32/package_esp32_index.json

3. 工具 → 开发板 → 开发板管理器 → 搜索并安装ESP32

4. 安装所需库：

  ◦ 工具 → 管理库 → 搜索并安装：

    ▪ Adafruit NeoPixel

    ▪ MPU6050_light

    ▪ ESP32Servo（可选）

硬件组装步骤

步骤1：焊接电路板

1. 准备万用板或定制PCB

2. 按顺序焊接：

  ◦ 电源模块（TP4056 + 升压模块）

  ◦ ESP32开发板

  ◦ 传感器模块（MPU6050，HC-SR04）

  ◦ 按钮和指示灯

3. 测试各模块电压

步骤2：连接LED灯带

# WS2812B连接方式
ESP32 GPIO4 → 灯带数据输入(DIN)
5V电源正极 → 灯带VCC
电源负极 → 灯带GND

# 注意：在VCC和GND之间并联1000μF电容


步骤3：外壳组装

1. 3D打印外壳部件（文件位于/hardware/3d_models/）

  ◦ 主体外壳

  ◦ 透明灯罩

  ◦ 安装支架

2. 使用M3螺丝固定PCB

3. 粘贴LED灯带

4. 安装防水胶圈和密封胶

首次配置

// 在config.h中调整参数
#define DEFAULT_BRIGHTNESS  100   // 默认亮度 (0-255)
#define BRAKE_SENSITIVITY   3.0f  // 刹车灵敏度
#define LOW_BATTERY_LEVEL   3.3f  // 低电量电压


📖 使用指南

基本操作

按钮功能

按钮	短按	长按（2秒）	双击
模式	切换灯光模式	进入校准模式	-
亮度	循环亮度等级	切换自动亮度	-
电源	显示电量	开关机	锁定/解锁

灯光模式说明

1. 常亮模式：恒亮红灯，日常使用

2. 呼吸模式：柔和呼吸效果，夜间使用

3. 闪烁模式：快速闪烁，提高注意力

4. 彩虹模式：多彩流动，装饰效果

5. 跑马模式：流动灯光，侧面增强

6. 反应模式：根据环境变化

7. 安全模式：全自动智能调节

校准流程

MPU6050加速度计校准

1. 将自行车置于水平地面
2. 长按模式按钮3秒进入校准模式
3. LED闪烁蓝色，保持静止5秒
4. 校准完成，LED变绿
5. 短按按钮退出


超声波传感器校准

1. 在距离传感器1米处放置参考物体
2. 进入校准模式（长按模式+亮度按钮）
3. 听到确认音后完成校准


移动应用控制

Android应用设置

1. 从Releases页面下载APK文件
2. 在手机上安装并打开应用
3. 打开手机蓝牙
4. 在应用中搜索设备"ESP32-BikeLight"
5. 配对连接（密码：123456）
6. 开始远程控制


应用功能

• 实时模式切换

• 亮度精细调节

• 查看电池状态

• 设置地理围栏

• 查看骑行统计

• 固件OTA更新

🧪 测试与验证

单元测试

# 运行测试套件
pio test

# 测试覆盖率报告
pio test --coverage

# 测试特定模块
pio test -e test_sensors
pio test -e test_led_effects


功能测试清单

电气测试

• [ ] 输入电压：5V ± 5%

• [ ] 待机电流：< 10mA

• [ ] 工作电流：< 200mA

• [ ] 充电电流：500mA ± 10%

传感器测试

• [ ] MPU6050：倾斜检测 ±0.5°精度

• [ ] HC-SR04：距离测量 2-200cm范围

• [ ] 光敏电阻：光照检测 0-100k Lux

• [ ] 温度监测：-10°C ~ 60°C

性能测试

• [ ] 刹车响应时间：< 100ms

• [ ] 灯光更新频率：30 FPS

• [ ] 蓝牙连接延迟：< 50ms

• [ ] GPS定位精度：< 5米

环境测试

# 温度循环测试 (-10°C ↔ 60°C, 5 cycles)
# 湿度测试 (95% RH, 24小时)
# 振动测试 (10-500Hz, 1小时)
# 防水测试 (IP65标准)


🐛 故障排除

常见问题与解决方案

问题现象	可能原因	解决方案
LED灯完全不亮	电源未接通
数据线连接错误	检查电池连接
确认GPIO4连接正确
测量5V输出
部分LED不亮或颜色异常	数据信号衰减
电源不足	检查数据线连接
在VCC-GND间加电容
缩短灯带长度
刹车检测不灵敏	MPU6050安装不水平
阈值设置过高	重新校准加速度计
降低BRAKE_SENSITIVITY值
超声波测距不准	传感器表面有污垢
环境干扰	清洁传感器表面
添加软件滤波
避免雨天使用
电池续航时间短	LED亮度设置过高
电池老化	降低默认亮度
启用自动亮度
更换新电池
蓝牙连接不稳定	距离过远
信号干扰	确保在10米范围内
远离WiFi路由器
重启ESP32蓝牙

错误代码参考

// 系统错误代码
#define ERROR_BATTERY_LOW      0x01  // 电池电压低
#define ERROR_SENSOR_MPU       0x02  // MPU6050故障
#define ERROR_SENSOR_SONIC     0x03  // 超声波故障
#define ERROR_LED_COMM         0x04  // LED通信错误
#define ERROR_OVER_TEMP        0x05  // 温度过高
#define ERROR_MEMORY_FULL      0x06  // 存储空间满

// LED闪烁模式表示错误
// 快速红闪2次: ERROR_BATTERY_LOW
// 红黄交替闪: ERROR_SENSOR_MPU
// ...等等


调试模式

// 启用调试输出
#define DEBUG_SERIAL       1
#define DEBUG_BRAKE        1
#define DEBUG_DISTANCE     0
#define DEBUG_POWER        1

// 串口输出示例
// [BRAKE] Acceleration: 3.5 m/s² (BRAKING!)
// [POWER] Battery: 3.8V (75%)
// [LED] Mode changed to: Breathing


📊 性能规格

电气特性

参数	最小值	典型值	最大值	单位
输入电压	3.0	3.7	4.2	V
工作电流 (待机)	-	8	12	mA
工作电流 (常亮)	-	120	180	mA
工作电流 (闪烁)	-	150	220	mA
充电电流	450	500	550	mA
工作温度	-10	25	60	°C

运行时间估算 (2600mAh电池)

灯光模式	亮度等级	估算运行时间
常亮模式	低 (30)	21小时
常亮模式	中 (100)	8小时
常亮模式	高 (255)	4小时
呼吸模式	自动	32小时
闪烁模式	中	17小时
深度睡眠	-	200小时

传感器性能

传感器	测量范围	精度	响应时间
MPU6050	±16g, ±2000°/s	0.5°	<10ms
HC-SR04	2cm-4m	±0.3cm	<60ms
光敏电阻	0-100k Lux	±10%	<100ms
温度监测	-40°C~125°C	±0.5°C	<1s

📁 项目结构

esp32-bike-tail-light/
├── firmware/                    # ESP32固件
│   ├── src/                    # 源代码
│   │   ├── main.cpp           # 主程序入口
│   │   ├── light_effects/     # 灯光效果模块
│   │   │   ├── effects.cpp
│   │   │   ├── effects.h
│   │   │   └── patterns.cpp
│   │   ├── sensors/           # 传感器模块
│   │   │   ├── mpu6050.cpp
│   │   │   ├── ultrasonic.cpp
│   │   │   └── sensor_fusion.cpp
│   │   ├── power/             # 电源管理
│   │   │   ├── battery.cpp
│   │   │   ├── charging.cpp
│   │   │   └── sleep_mode.cpp
│   │   ├── communication/     # 通信模块
│   │   │   ├── bluetooth.cpp
│   │   │   ├── wifi_ota.cpp
│   │   │   └── gps.cpp
│   │   ├── ui/                # 用户界面
│   │   │   ├── buttons.cpp
│   │   │   ├── buzzer.cpp
│   │   │   └── indicators.cpp
│   │   └── utils/             # 工具函数
│   │       ├── config.cpp
│   │       ├── logger.cpp
│   │       └── filters.cpp
│   ├── include/               # 头文件
│   │   ├── config.h          # 配置文件
│   │   ├── pins.h            # 引脚定义
│   │   └── constants.h       # 常量定义
│   └── platformio.ini        # PlatformIO配置
│
├── hardware/                  # 硬件设计
│   ├── schematics/           # 电路原理图
│   │   ├── main_schematic.pdf
│   │   ├── power_schematic.pdf
│   │   └── pcb_layout.pdf
│   ├── pcb/                  # PCB设计文件
│   │   ├── gerber_files/    # 生产文件
│   │   ├── bom.csv          # 物料清单
│   │   └── assembly.pdf     # 组装图
│   ├── 3d_models/           # 3D模型文件
│   │   ├── case_body.stl
│   │   ├── case_cover.stl
│   │   ├── mount_bracket.stl
│   │   └── enclosure.step
│   └── datasheets/          # 元件数据手册
│
├── mobile_app/               # 移动应用程序
│   ├── android/             # Android应用
│   │   ├── app/
│   │   ├── build.gradle
│   │   └── README.md
│   ├── ios/                 # iOS应用
│   │   ├── BikeLight/
│   │   ├── Podfile
│   │   └── README.md
│   └── flutter/             # Flutter跨平台
│       ├── lib/
│       ├── pubspec.yaml
│       └── README.md
│
├── docs/                     # 文档
│   ├── user_manual.pdf      # 用户手册
│   ├── technical_manual.pdf # 技术手册
│   ├── assembly_guide.pdf   # 组装指南
│   ├── api_reference.md     # API参考
│   └── images/              # 图片资源
│
├── tests/                    # 测试文件
│   ├── unit_tests/          # 单元测试
│   │   ├── test_effects.cpp
│   │   ├── test_sensors.cpp
│   │   └── test_power.cpp
│   ├── integration_tests/   # 集成测试
│   └── performance_tests/   # 性能测试
│
├── scripts/                  # 脚本工具
│   ├── build.py             # 构建脚本
│   ├── flash.py             # 烧录脚本
│   ├── test.py              # 测试脚本
│   └── calibrate.py         # 校准工具
│
├── .github/                  # GitHub配置
│   ├── workflows/           # CI/CD工作流
│   │   ├── build.yml
│   │   ├── test.yml
│   │   └── release.yml
│   └── ISSUE_TEMPLATE/      # Issue模板
│
├── .vscode/                  # VSCode配置
│   ├── extensions.json
│   └── settings.json
│
├── .gitignore
├── LICENSE
├── README.md                 # 本文档
├── CHANGELOG.md             # 更新日志
└── CONTRIBUTING.md          # 贡献指南


🤝 贡献指南

我们欢迎并感谢所有形式的贡献！以下是参与本项目的方式：

开发流程

1. Fork 仓库并克隆到本地

2. 创建分支：git checkout -b feature/YourFeature

3. 提交更改：git commit -m 'Add some feature'

4. 推送到分支：git push origin feature/YourFeature

5. 提交 Pull Request

代码规范

// 1. 命名规范
class BikeTailLight {          // 类名：驼峰式，首字母大写
    void updateSensors();      // 方法名：驼峰式，首字母小写
    int led_brightness;        // 变量名：下划线分隔
    const int MAX_BRIGHTNESS;  // 常量：全大写，下划线分隔
};

// 2. 注释规范
/**
 * @brief 更新所有传感器数据
 * @param force 是否强制更新（忽略时间间隔）
 * @return 更新是否成功
 */
bool updateSensorData(bool force = false) {
    // 单行注释使用 //
    if (!initialized && !force) {
        return false;  // 简短的尾随注释
    }
    // ... 具体实现
}


提交信息格式

类型(范围): 简短描述

详细描述（可选）

脚注（可选，如修复的issue编号）

类型说明：
- feat: 新功能
- fix: 修复bug
- docs: 文档更新
- style: 代码格式
- refactor: 重构
- test: 测试相关
- chore: 构建过程或工具更新


报告问题

当报告问题时，请包括：

1. 问题描述

2. 复现步骤

3. 预期行为

4. 实际行为

5. 环境信息（固件版本、硬件版本等）

6. 相关日志或截图

📄 许可证

本项目采用 MIT 许可证 - 查看 LICENSE(LICENSE) 文件了解详细信息。

MIT License

Copyright (c) 2024 Your Name

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


📞 支持与联系

获取帮助

• 📖 文档：项目Wiki页面(https://github.com/你的用户名/esp32-bike-tail-light/wiki)

• 🐛 问题反馈：GitHub Issues(https://github.com/你的用户名/esp32-bike-tail-light/issues)

• 💬 讨论区：GitHub Discussions(https://github.com/你的用户名/esp32-bike-tail-light/discussions)

社区渠道

• QQ群：123456789（ESP32自行车项目交流）

• Discord：加入链接(https://discord.gg/你的服务器)

• B站：视频教程(https://space.bilibili.com/你的ID)

联系开发者

• 邮箱：your.email@example.com

• 个人网站：https://yourwebsite.com(https://yourwebsite.com)

• Twitter：@yourusername(https://twitter.com/yourusername)

支持项目

如果这个项目对你有帮助，请考虑：

1. ⭐ 给项目点个Star

2. 🐛 报告遇到的问题

3. 🔧 提交代码改进

4. 📖 改进文档

5. 💬 分享给其他人


------

<p align="center">

安全骑行，智慧照明

</p>

<p align="center">

基于ESP32的智能自行车尾灯 | 开源硬件项目 | 更新于 2024年6月

</p>

⚠️ 安全须知

重要安全警告

1. 电气安全

  ◦ 仅使用带保护板的18650锂电池

  ◦ 避免在潮湿环境中充电

  ◦ 定期检查线路绝缘状况

2. 骑行安全

  ◦ 本产品为辅助安全设备，不能替代主动安全意识

  ◦ 确保灯光安装牢固，不会在骑行中脱落

  ◦ 遵守当地交通法规对自行车灯光的要求

3. 使用环境

  ◦ 工作温度：-10°C ~ 60°C

  ◦ 防护等级：IP65（正确安装情况下）

  ◦ 避免长时间阳光直射

免责声明

本项目为开源硬件/软件项目，使用者需自行承担风险。开发者不对因使用本项目造成的任何直接或间接损失负责。请在理解所有安全风险的前提下使用本产品。

法规符合性

• 灯光颜色：红色（符合自行车尾灯标准）

• 闪烁频率：1-4Hz（符合EN15194标准）

• 亮度等级：可调节，符合不同环境需求


------

最后更新：2026年2月15日

版本：v1.0.0

维护者：Sparkx(https://github.com/henaer)