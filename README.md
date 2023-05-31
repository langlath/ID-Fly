### Welcome to ID-Fly-Novabot project
ID-Fly is a research project about Novabot, a miniaturized blimp for artistic purposes. 

## 3D Model
Most of the blimp structure is 3D-printed. You will find already existing files of the pieces in the 3D printed pieces folder. You can use softwares such as Fusion 360 to open them.

## Raspberry Pi
The project uses a Raspberry Pi Zero W in order to get a video stream from a camera, setting up an html page to pilot the blimp and commanding the motors.

# Test RPi
The RPi used for testing (and not mounted on the blimp) is a Zero W as well, with a default 32 bit Debian OS with bullseye. Its user name and password are novapi. When accessing it by ssh, you may want to precise you need to create opencv windows : 
```bash
ssh -Y novapi@<ip address>
```
When on the EMSE-INVITE Wifi network, ip address is often 192.168.164.40 or 192.168.41.

