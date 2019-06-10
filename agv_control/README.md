# agv_control demo project
# 说明
该项目为示例项目, 展示说明各个功能函数的使用方法.

# 配置
迈信EP3L低压交流伺服电机驱动器, 采用485总线和modbus_rtu协议进行通信.

|    配置变量    |     参数     |               说明                |
| :------------: | :----------: | :-------------------------------: |
|     波特率     |    57600     | 最高115200通讯不成功, 故采用57600 |
|     串口号     | /dev/ttyUSB0 |  未进行串口绑定,需自行查看端口号  |
|   锂电池电压   |     24V      |        实际电压约29.5V左右        |
| 逆变器输出电压 |  220V 50HZ   |     电池输出一路直接接逆变器      |

# 依赖项安装
ros下的serial库的安装
```
sudo apt-get install ros-kinetic-serial
```
其中kinetic可更换成使用的ROS版本.

# 宏定义及函数说明
## 宏定义
| 宏定义名称  | 宏定义值 | 宏定义意义                               |
| :---------- | :------- | :--------------------------------------- |
| \_VERSION\_ | 1.0      | 当前头文件版本号                         |
| A_ADD_B     | 0.5365   | 车体的a+b，其中a=0.2365，b=0.3，单位：米 |
| K_VTOW      | 4900     | 线速度转角速度的比例值×减速比，即总比值  |
| WHEEL_FL    | 2        | 前左轮                                   |
| WHEEL_FR    | 3        | 前右轮                                   |
| WHEEL_BL    | 5        | 后左轮                                   |
| WHEEL_BR    | 4        | 后右轮                                   |
| ENABLE      | 1        | 驱动器使能开启                           |
| DISABLE     | 0        | 驱动器使能关闭                           |

## 函数集
| 函数名                        | 参数                               | 说明                   | 功能                 |
| :---------------------------- | :--------------------------------- | :--------------------- | :------------------- |
| [MotorControl](#MotorControl) | char* device <br>uint32_t baudrate | 设备串口号<br>波特率   | 构造函数, 对象初始化 |
| [wheelEnable](#wheelEnable)   | uint32_t wheelId<br>int32_t enable | 麦轮的编号<br>使能信号 | 使能指定轮子的驱动器 |
|[activate](#activate)|int32_t wheelId|麦轮编号|生效传入指定驱动器的参数|参数设置完成之后需执行此函数,否则参数不会生效|
|[activateAll](#activateall)|无|无|激活所有驱动器传入的参数|
|[setWheelSpeed](#setwheelspeed)|int32_t wheelId<br>int32_t speed|麦轮编号<br>麦轮速度|设置指定麦轮的转速|
|[setWheelSpeedAll](#setwheelspeedall)|int32_t speed_fl<br>int32_t speed_fr<br>int32_t speed_bl<br>int32_t speed_br|前左轮转速<br>前右轮转速<br>后左轮转速<br>后右轮转速|同上|
|[setSpeedXYW](#setspeedXYW)|double vx<br>double vy<br>double w|正前方线速度<br>向左线速度<br>向左转角速度|采用XYW的速度控制方式|
|[readWheelSpeed](#readwheelspeed)|int32_t wheelId<br>int16_t*speed|麦轮编号<br>获取的转速存储在该变量中|获取指定麦轮的转速|

### MotorControl
该函数为构造函数,需要传入串口号和波特率,其中串口号可以采用
### wheelEnable
### activate
### activateAll
### setWheelSpeed
### setWheelSpeedAll
### setSpeedXYW
### readWheelSpeed
### ~MotorControl
