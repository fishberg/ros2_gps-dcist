# GPS DCIST

## Clone
```
mkdir -p ~/ws/src
cd ~/ws/src
git clone --recurse-submodules git@github.com:fishberg/ros2_gps-dcist.git gps-dcist
git clone git@github.com:MIT-SPARK/ROS-System-Monitor.git
```

## Install / Config
```
sudo apt install -y gpsd gpsd-tools gpsd-clients rtklib jq libasio-dev
sudo systemctl stop gpsd
sudo systemctl stop gpsd.socket
sudo systemctl disable gpsd
sudo usermod -aG dialout $USER
reboot

# confirm /dev/gps0 shows up when GPS is plugged in
ls -l /dev/gps*
```

## Build
```
colcon build --symlink-install
source ./install/setup.bash
```

## Run
```
ros2 launch gps_dcist gps_dcist.launch.yaml \
    launch_gps:=true \
    launch_gps_monitor:=true \
    launch_ntrip:=true \
    launch_ntrip_monitor:=true
```

```
ros2 launch gps_dcist gps_dcist.launch.yaml launch_gps:=true launch_gps_monitor:=true launch_ntrip:=true launch_ntrip_monitor:=true
```

```
ros2 launch gps_dcist gps_dcist.launch.yaml launch_gps:=true launch_ntrip:=true
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
