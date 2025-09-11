# 标准消息库

## 标准消息

### Pose2D
数据结构：
```cpp
typedef struct
{
    float x;
    float y;
    float theta;
}Pose2D;
```

### Kinematic2D
数据结构：
```cpp
typedef struct
{
    Pose2D pos;
    float vx;
    float vy;
    float vtheta;
}Kinematic2D;
```

### Command
数据结构：
```cpp
typedef struct
{
    float linear_x;
    float linear_y;
    float angular_z;
    uint8_t control_mode; // 0: stop, 1: manual, 2: auto    
}Command;
```
