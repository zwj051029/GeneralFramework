# Ws2812驱动库

## 简介
本库为原库 [WS2812_yx]() 的升级移植版本。从C移植到了C++，并且改为了使用PWM驱动，以适配各MCU不同的主频

本库的普通使用方式基本和原库一致，计划新增配合新的RobotSystem的使用方式，会在后续更新

> LastUpdate: 2025-9-10 $~~~~~~~~~~~~~$ ——*Huangney*

## 快速开始
### 1. 配置PWM外设
在`CubeMX`中启用一个TIM的PWM功能 并配置好对应的引脚

将其的输出频率配置为 ***800KHz***

### 2. 引入头文件
```cpp
#include "led_ws2812.hpp"
```
### 3. 创建对象并初始化
```cpp
LedWs2812 led_ws2812;  // 实例化调用空构造函数，这是为了防止在HAL库之前调用
led_ws2812.Init(&htimX, TIM_CHANNEL_Y);        // X为TIM编号 Y为通道编号            
```
### 4. 维护灯带管理器
在一个线程中循环调用实例的 `Handle` 方法
```cpp
void Task
{
    ...
    while (1)
    {
        led_ws2812.Handle();
        osDelayUntil(...);
    }
}
```

## 普通使用方式
使用本库的普通使用方式和原库一致

通过注册灯带状态并绑定状态布尔变量，即可控制灯带的显示


## 系统耦合方式
需要调用最开始实例化的 `RobotSystem` 的方法 `LedDispRegist` 来初始化

完成注册之后，系统会自动更新灯带的状态