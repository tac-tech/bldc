# CAN Documentation with Canable + VESC

You can find more about the canable here: https://canable.io/getting-started.html
## Linux
### Setup:
You must have socketCAN installed on your linux machine:
``` bash
 sudo apt install can-utils
```

To enable SocketCAN, you must enable the relevant linux kernel modules:
``` bash
sudo modprobe can
sudo modprobe vcan
sudo modprobe slcan
```
Source: https://github.com/gribot-robotics/documentation/wiki/Installing-SocketCAN
