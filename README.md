# Welcome to ID-Fly-Novabot project
ID-Fly is a research project about Novabot, a miniaturized blimp for artistic purposes. 

## 3D Model
Most of the blimp structure is 3D-printed. You will find already existing files of the pieces in the 3D printed pieces folder. You can use softwares such as Fusion 360 to open them.

## Raspberry Pi
The project uses a Raspberry Pi Zero W in order to get a video stream from a camera, setting up an html page to pilot the blimp and commanding the motors.

### Test RPi
The RPi used for testing (and not mounted on the blimp) is a Zero W as well, with a default 32 bit Debian OS with bullseye. Its user name and password are novapi. When accessing it by ssh, you may want to precise you need to create opencv windows : 
```bash
ssh -Y novapi@<ip address>
```
When on the EMSE-INVITE Wifi network, ip address is often 192.168.164.40 or 192.168.41.


## Simu and control
The blimp is simulated with v-rep. The latest scene is visual_and_bent_camera.ttt. It communicates with python algorithms via ROS. Before launching v-rep, please make sure ROS is launched. To do so, you must first source you ROS version :
```bash
source /opt/ros/<your ROS version >/setup.bash
```
Then you can launch ROS : 
```bash
roscore
```

Next, in another terminal, source ROS again : 
```bash
source /opt/ros/<your ROS version >/setup.bash
```
Now you can launch v-rep.

Once you have loaded the desired scene and you are playing it, you must run the python programs. First go to your ROS workspace : 
```bash
cd ws_ros/ros
```
Source ROS : 
```bash
source /opt/ros/<your ROS version >/setup.bash
```
Then build your project : 
```bash
catkin_make
```
Source what you just built : 
```bash
source devel/setup.bash
```
Now you can run your program :
```bash
rosrun simu vrep_interm.py
```

To run your second program, open a second terminal and source again : 
```bash
source /opt/ros/<your ROS version >/setup.bash
```
```bash
source devel/setup.bash
```
And now you can run the second program : 
```bash
rosrun simu control.py
```
## Flight Recordings
https://drive.google.com/drive/folders/15XXGsDaq2065NMfhoMPHFT2Ng_2tn5A1?usp=sharing
