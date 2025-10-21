# GPS DCIST

## Clone
```
cd ~/ws/src
git clone --recurse-submodules git@github.com:fishberg/ros2_gps-dcist.git gps-dcist
git clone git@github.com:MIT-SPARK/ROS-System-Monitor.git
```

## Install
```
sudo apt install -y libasio-dev
```

## NTRIP Testing
```
# write to log file
str2str -in ntrip://user:pass@caster.example.com:2101/MOUNT -out file://rtcm_log.rtcm3

# write to serial
str2str -in ntrip://username:password@caster.domain.com:2101/MOUNTPOINT -out serial://ttyACM0:115200:8:n:1
```

```
# write upenn data to file
str2str -in ntrip://dcist:Fernando@192.168.129.25:2101/U-BLOX -out file://rtcm_log.rtcm3

# write upenn to gps0
str2str -in ntrip://dcist:Fernando@192.168.129.25:2101/U-BLOX -out serial://gps0:9600:8:n:1
str2str -in ntrip://dcist:Fernando@192.168.129.25:2101/U-BLOX -out serial://gps0
```

```
# run gpsd with ntrip
gpsd ntrip://dcist:Fernando@192.168.129.25:2101/U-BLOX
```

```
# check which /dev/gpsX -> /dev/ttyACMX
ll /dev/gps*
```
