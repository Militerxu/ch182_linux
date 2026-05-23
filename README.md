# CH182 PHY 设备树绑定文档

本文档描述 **WCH** 开发的 CH182 系列以太网 PHY 的设备树绑定。

## 支持的 PHY 型号

- CH182D
- CH182F2
- CH182F7
- CH182F8
- CH182H1
- CH182H2
- CH182H3
- CH182H6
- CH182H7
- CH182H8

## LED 模式配置

CH182 PHY 支持两种 LED 配置模式：**传统模式**和**自定义模式**。

> 注意：同一时间只能使用其中一种模式。请不要在设备树中同时配置传统 LED 模式和自定义 LED 模式。

### 1. 传统 LED 模式

使用传统 LED 配置时，模式通过 Page 7 寄存器 `0x13` 的 `[5:4]` 位进行设置。该配置适用于所有 CH182 系列 PHY。

如需通过设备树启用该模式，请使用：

```dts
wch,traditional-led = <1>; // Mode value (0-3), corresponds to bits [5:4] in register 0x13
```

有效的传统 LED 模式取值及其含义，请参考 CH182DS2 数据手册。

### 2. 自定义 LED 模式

在自定义 LED 配置模式下，模式通过 Page 7 寄存器 `0x11` 进行设置：

- `bit[7:4]`：LED0
- `bit[3:0]`：LED1

请在设备树中使用以下属性进行配置：

```dts
wch,led0-mode = <1>; // Set LED0 mode: binary 0b0001
wch,led1-mode = <7>; // Set LED1 mode: binary 0b0111
```

> 取值必须是 `<0>` 到 `<7>` 范围内的整数，表示 4 位二进制模式设置。例如，`<7>` 表示 `0b0111`。

有效的模式取值和说明，请参考 [LED 配置表](https://www.cnblogs.com/llidd/p/18748573) 或 **CH182DS2 数据手册**。

## 时钟配置

### Clocks 和 Clock Names

CH182 PHY 支持 RMII 参考时钟输入，用于确定 XI 输入时钟。请使用通用时钟绑定配置 `clocks` 和 `clock-names`：

```dts
clocks = <&clks IMX6UL_CLK_ENET_REF>;
clock-names = "rmii-ref";
```

## 设备树示例

```dts
&fec1 {
    pinctrl-names = "default";
    pinctrl-0 = <&pinctrl_enet1>;
    phy-mode = "rmii";
    phy-handle = <&ethphy1>;
    status = "okay";

    mdio: mdio {
        #address-cells = <1>;
        #size-cells = <0>;

        ethphy1: ethernet-phy@1 {
            compatible = "ethernet-phy-ieee802.3-c22";
            reg = <1>; // MDIO bus address
            // wch,traditional-led = <1>; // Use either this (0-3)
            // wch,led0-mode = <3>;   // or these two (0-15, 4-bit binary), use either this (0-7)
            // wch,led1-mode = <4>;  // Use either this (0-7)
            clocks = <&clks IMX6UL_CLK_ENET_REF>;
            clock-names = "rmii-ref";
        };
    };
};
```

---

更多寄存器配置细节和完整模式定义，请参考官方 CH182 数据手册。

如有任何问题，可以发送反馈邮件至：tech@wch.cn
