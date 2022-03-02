# INTRODUCTION
In normal conditions, we always use ros package universal_robot to startup the ur robot. It provide a lot of function. However, sometimes we just want to startup the robot, and send the script program to robot, it is not necessary to start a lot of program. So, I create this project to provide a simplified interface which is not depend on ROS.
# Document
## urdashboard.py
### describe
This python file provide a simple way to connect the dashboard, send the dashboard command to the robot.
These command can control the robot system include power on/off robot, load the exist program etc.

More command can refer to [Dashboard_server_CE-Series.pdf](./doc/Dashboard_Server_CB-Series.pdf)

### example
```
from pyurrobot import urdashboard

dashboard = urdashboard.URDashboard('192.168.1.12')
print(dashboard.sendCommand('power on'))

# release the object
del dashboard
```

## RG_gripper.py
### describe
This python file help us to control the RG2, include the open width of RG2 and the force.

### example
```
import urx
import RG_gripper as rg

rob = urx.Robot("192.168.1.19")
rg2 = rg.RG2(rob)
rg2.setWidth(width=80) # 80mm width
```


