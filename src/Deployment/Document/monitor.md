### Master/Device:

**termianl one**: roscore

**terminal two**: python3 device.py

**terminal three:**

​		export ROS_IP=*master_ip*

​		export ROS_MASTER_URI=[http://*master_ip*:11311](http://master_ip:11311)

​		python talker.py (whatever program that is publishing)





### Slave/Controller :

**terminal one**:

​		export ROS_IP=slave_ip

​		python monitor.py





—————————————————————





**problem**: once the topic stop, no recover

**problem**: only one topic per machine
