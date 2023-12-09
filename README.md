# lane_following_rc_car

Lane following RC car for ELEC 424 final project\
Team members: Jun Chu, Rachel Gu, Wendy Tan, Chris Zhou\


# Instructions to run
Compile the device tree overlay file\
```~ $ dtc -@ -I dts -O dtb -o spd.dtbo spd.dts```

Copy the compiled file to the overlays directory\
```~ $ cp spd.dtbo /boot/overlays/```

Add the following line to the end /boot/config.txt. You must reboot after this step\
```~ $ dtoverlay=name_of_file```

Now compile the speed encoder driver, and insert the compiled module\
```~ $ make```\
```~ $ sudo mkdir /lib/modules/$(uname -r)/misc/```\
```~ $ sudo cp spd_encoder_driver.ko /lib/modules/$(uname -r)/misc/```\
```~ $ sudo depmod```\
```~ $ sudo modprobe spd_encoder_driver```

To remove the module,\
```~ $ sudo modprobe -r spd_encoder_driver```

Finally, run the main python file\
```~ $ export LD_PRELOAD=/usr/lib/arm-linux-gnueabihf/libatomic.so.1```\
```~ $ python3 final.py```
