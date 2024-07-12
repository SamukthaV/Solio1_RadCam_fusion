# User Manual for Radar and Camera-based Collision Warning System

### *This instruction manual helps users run the Collision Warning System in SOLIO. The sensors and the connections to ORIN are shown below:*
<img src="./img/hardware.png" alt="Hardware image" width="400"> 

## 1. Configuring the RADAR and Camera Sensors:

### Camera
1. [Install PylonViewer](https://www.baslerweb.com/en/products/) software based on the camera model to access and control the settings of the Basler camera.
2. The camera model used in the project is **acA1920-40uc**.
3. Set the frame rate and the autofocus mode on the PylonViewer as shown below.
   
   <img src="./img/pylon.png" alt="Pylonviewer" width="400"> 

### Radar
1. Configure the radar sensor as shown below:
   
   <img src="./img/radar_wired.png" alt="RADAR wired settings" width="400"> 

## 2. Data Acquisition from Sensors

### Steps
1. To acquire data from Radar Front/Rear (depending on the application) and Camera.
2. Follow the steps based on the type of warning:
   - [Front](#to-perform-forward-collision-warning-system)
   - [Rear](#to-perform-rear-collision-warning-system)

## Enabling Radar Acquisition System
The common steps involved in enabling the radar system are as follows:

1. Enabling the Perception Development Kit (PDK):
    ```bash
    cd /opt/pdk/bin
    ./pdk_start.sh
    ```
   *The Long-range radar ARS430DI will be automatically enabled when the above command is run. To receive data from the Short-range radar, run the below commands.*

2. Set up CAN0 for the left Short-range RADAR and CAN1 for the Right Short-range RADAR mounted on the vehicle. If prompted for a password, provide the system admin password.
    ```bash
    sudo ip link set can0 up type can bitrate 500000 dbitrate 2000000 fd on
    sudo ip link set can1 up type can bitrate 500000 dbitrate 2000000 fd on
    ```

   The following is the reference image for PDK:

   <img src="./img/pdk.png" alt="PDK Image" width="400"> 

## Publishing Radar Data 
1. To publish Front long-range radar:
    ```bash
    cd Downloads/radar_ros/src/conti_radar/_build/devel/lib/conti_radar/
    ./lrr_front_obj
    ```
2. To publish Rear long-range radar:
    ```bash
    cd Downloads/radar_ros/src/conti_radar/_build/devel/lib/conti_radar/
    ./lrr_rear_obj
    ```
3. To publish Left short-range radar:
    ```bash
    cd Downloads/radar_ros/src/conti_radar/_build/devel/lib/conti_radar/
    ./srr_left_obj
    ```
4. To publish Right short-range radar:
    ```bash
    cd Downloads/radar_ros/src/conti_radar/_build/devel/lib/conti_radar/
    ./srr_right_obj
    ```

## Speed Adjustment Algorithm

The algorithm ensures the ego vehicle adjusts its speed based on the movement of an obstacle vehicle in an adjacent lane. It operates based on distances between the obstacle and ego vehicles. The ego vehicle's speed is reduced when an obstacle vehicle crosses into its lane and remains unchanged when the obstacle stays in its own lane. The code executes commands to adjust the vehicle's speed appropriately when a vehicle cuts in or out in either direction.

### Commands
- **slowdown**
  - Reduces the vehicle's speed to half.
- **go**
  - Maintains the vehicle's current speed.

The following code snippet demonstrates how the filtering algorithm operates and publishes commands based on the obstacle vehicle's actions:

```bash
cd Downloads/radar_ros/src/conti_radar/src
python3 srr_right_updated.py

```bash
cd Downloads/radar_ros/src/conti_radar/src
python3 srr_left_updated.py
```


## To perform Forward Collision Warning System
1. Run the Python file [publishing front camera data](https://github.com/SamukthaV/Solio1_RadCam_fusion/blob/main/Collision%20warning%20based%20on%20Sensor%20fusion/FCWS%20%2B%20cut-in%20%2B%20cut-out/front_cam_pub.py)
```bash
source fusion/bin/activate
python3 front_cam_pub.py
```
<img src="./img/frontcampub.png" alt="front camera data" width="400"> 
2. Source the radar functions by running the following command

```bash
      source /home/orin/Downloads/radar_ros/src/conti_radar/_build/devel/setup.bash
```
3.  Run the Python file for 
[fusing radar and camera data](https://github.com/SamukthaV/Solio1_RadCam_fusion/blob/main/Collision%20warning%20based%20on%20Sensor%20fusion/FCWS%20%2B%20cut-in%20%2B%20cut-out/front_radcam_fusion.py)

```bash
source fusion/bin/activate
python3 front_radcam_fusion.py
```
<img src="./img/frontfusion.png" alt="Pylonviewer" width="400"> 


## To perform Rear Collision Warning System
1. Run the Python file [publishing rear camera data](https://github.com/SamukthaV/Solio1_RadCam_fusion/blob/main/Collision%20warning%20based%20on%20Sensor%20fusion/RCWS/Rear_cam_pub_.py)
```bash
source fusion/bin/activate
python3 Rear_cam_pub.py
```
<img src="./img/rearcampub.png" alt="Rear camera publishing" width="400"> 

3. Source the radar functions by running the following command
   ```bash
   source /home/orin/Downloads/radar_ros/src/conti_radar/_build/devel/setup.bash

    ```
4.  Run the python file [fusing rear radar and camera data](https://github.com/SamukthaV/Solio1_RadCam_fusion/blob/main/Collision%20warning%20based%20on%20Sensor%20fusion/RCWS/Rear_radcam_fusion.py)
```bash
source fusion/bin/activate
python3 Rear_radcam_fusion.py
```
<img src="./img/rearfusion.png" alt="rear fusion" width="400"> 

6. To integrate all the commands from SRR of both the sides and LRR of front and rear [integrated vehicle commands](https://github.com/SamukthaV/Solio1_RadCam_fusion/blob/main/Collision%20warning%20based%20on%20Sensor%20fusion/FCWS%20%2B%20cut-in%20%2B%20cut-out/integrate%20sensor%20commands.py)
```bash
python3 integrate_sensor_commands.py
```
