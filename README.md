the first : 

insert usb2can device, run : 

```
$ lsusb
$ sudo ip addr
```

to check can status.

the second : 

load mods can requires and  check: 

```
$ sudo modprobe can
$ sudo modprobe can_raw
$ sudo modprobe can_dev

$ lsmod | grep can
```

the third : 

install can-utils : 

```
$ git clone https://github.com/linux-can/can-utils.git
$ cd can-utils 
$ make
$ sudo make install
```

the fourth : 

run can0.sh

```
$ ./can0.sh
```

