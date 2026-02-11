# xvc-esp32

这是一个在 ESP32 上运行的 Xilinx Virtual Cable (XVC) 服务器实现，允许通过 WiFi 将 Vivado 等工具连接到目标 FPGA 的 JTAG 端口进行编程与调试。

## 概要

- 本项目基于 Derek Mulcahy 的 Raspberry Pi 实现移植而来，已适配 ESP32 的 GPIO 与网络环境。
- 将 ESP32 的 GPIO 接到目标设备的 JTAG 引脚 (TDI / TDO / TMS / TCK)，主机端软件（例如 Vivado）可通过 `Add Virtual Cable` 将其识别为虚拟 JTAG 适配器。

![示例设备](picture.jpg)

## 与原版的主要差异说明

虽然本项目核心逻辑源自 `xvcpi` 和其他 XVC 实现，但针对具体使用场景做了以下重要修改：

1.  **集成串口透传功能 (Serial Bridge)**:
    -   原版仅作为 JTAG 适配器。
    -   **本项目新增**：在 `xvc-esp32.ino` 中集成了 `serialTask` 任务，实现了 USB Serial (电脑端) 与 UART1 (目标板端，GPIO 33/23) 的双向透传。
    -   **优势**：这意味着你只需要一个 ESP32 就可以同时实现 JTAG 调试和串口打印查看，无需额外的 USB 转 TTL 模块。

2.  **配置分离 (Configuration Separation)**:
    -   原版通常将 SSID 和密码硬编码在源文件中。
    -   **本项目改进**：将 WiFi 凭据分离到 `credentials.h` 文件中，避免在分享代码时泄露敏感信息。

3.  **移除特定硬件依赖**:
    -   原代码中注释掉了 `#include <M5Atom.h>`，使其不再依赖 M5Stack Atom 库，可用于通用的 ESP32 开发板。

4.  **静态 IP 支持**:
    -   代码中预留了 `USE_STATIC_IP` 宏，取消注释即可配置静态 IP 地址，方便在网络环境中固定调试器地址。

## 使用方法

1.  **配置 WiFi**:
    在 `credentials.h` 中填写你的 WiFi 名称与密码：
    ```cpp
    #define MY_SSID "your_ssid"
    #define MY_PASSPHRASE "your_password"
    ```

2.  **配置引脚**:
    根据你的硬件修改 `xvc-esp32.ino` 中的 JTAG 引脚定义，默认配置如下：
    ```cpp
    static constexpr const int tms_gpio = 22;
    static constexpr const int tck_gpio = 19;
    static constexpr const int tdo_gpio = 21;
    static constexpr const int tdi_gpio = 25;
    ```
    *注意：为了性能，建议使用 GPIO 0-31。*

    **串口引脚配置** (用于透传):
    ```cpp
    Serial1.begin(115200, SERIAL_8N1, 33, 23); // RX=33, TX=23
    ```

3.  **编译与烧录**:
    使用 Arduino IDE 或 `arduino-cli` 编译并将程序烧录到 ESP32。

4.  **连接 Vivado**:
    -   打开 Vivado Hardware Manager。
    -   选择 `Open Target` -> `Autoconnect` 或 `Add Virtual Cable`。
    -   输入 ESP32 的 IP 地址（串口监视器会打印，或者你设置的静态 IP），默认端口为 2542。

## 接线图

已在仓库中添加接线示意图：

![接线图](接线图.png)

**接线要点**：
-   ESP32 与目标 FPGA 开发板必须**共地 (GND)**。
-   TCK/TMS/TDI/TDO 分别连至目标设备对应的 JTAG 引脚。
-   若目标设备电平不匹配（如 1.8V FPGA IO），请务必使用电平转换器。

## 许可

本实现为衍生作品，原作者的实现采用 CC0 1.0（公共领域声明），本仓库沿用相同的许可条款。

/*
   This work, "xvc-esp32.ino", is a derivative of "xvcpi.c" (https://github.com/derekmulcahy/xvcpi)
   by Derek Mulcahy.

   "xvc-esp32.ino" is licensed under CC0 1.0 Universal (http://creativecommons.org/publicdomain/zero/1.0/)
   by Kenta IDA (fuga@fugafuga.org)
*/