## Installation on robot:

1. Install dependencies
```
sudo apt-get install python3-numpy python3-cffi python3-aiohttp \
    libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev \
    libswscale-dev libswresample-dev libavfilter-dev libopus-dev \
    libvpx-dev pkg-config libsrtp2-dev python3-opencv pulseaudio
```
2. Install rtcbot
```
sudo pip3 install rtcbot
```
 If the installation has breaked pip (pyoopenssl and cryptography error), tou can fix it by installing pyopenssl v22.0 or lower.
```
wget https://files.pythonhosted.org/packages/35/d3/d6a9610f19d943e198df502ae660c6b5acf84cc3bc421a2aa3c0fb6b21d1/pyOpenSSL-22.0.0.tar.gz

tar -xvzf pyOpenSSL-22.0.0.tar.gz

python3 setup.py install
```
 
3. Network configuration:

Configuration file in:

```
/etc/netplan/
```

File template (XY is the number of the robotont):

```
# Let NetworkManager manage all devices on this system
network:
  version: 2
  renderer: NetworkManager
  wifis:
    wlp58s0:
      optional: true
      addresses:
        - 192.168.200.XY/24
      access-points:
        "teleop5G":
           password: "teleop123"
      dhcp4: no
      gateway4: 192.168.200.1

#      dhcp4: no
#      addresses:
#        - 192.168.200.XY/24
#      access-points:
#        "robotont-16":
#          password: "---robotont passwd---"
#          mode: ap
```

Apply the configuration with:
```
sudo netplan apply
```

## Use:

1. Run python script (test-ros.py)

2. Go to 192.168.200.XY:8080 on a browser


## Testing on laptop with ROS:

1. Run roscore in separate terminal

2. Check speed values with
```
rostopic echo cmd_vel
```
