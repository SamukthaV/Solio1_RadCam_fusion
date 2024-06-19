#### *This instruction manual helps the users to run Collision warning system in SOLIO provided the sensors are mounted and the  connections are made as mentioned [here](https://github.com/SamukthaV/Solio1_RadCam_fusion/tree/main/Hardware%20setup%20)*

##### Steps
1) To acquire data from Radar Front/Rear (depending upon the application) and Camera
2) Follow the steps based on the type of warning ([Front](#section-2)/[Rear](#section-3))

## Enabling Radar Acquisition system
The common steps carried out in enabling the radar involves:
1) Opening the Perception Development Kit(PDK)
```bash
cd /opt/pdk/bin
./pdk_start.sh
```
The following is the reference image for pdk.

![pdk](./img/pdk.png)

#### * The Long range radar ARS430DI will be automatically enabled when we run the above command.Inorder to receive data from Short range radar run the below commands*
CAN0 is the communication channel for the left Short range RADAR and CAN1 for the Right Short range RADAR mounted on the vehicle.If requested for password provide the admin password of the system.
```bash
sudo ip link set can0 up type can bitrate 500000 dbitrate 2000000 fd on
sudo ip link set can1 up type can bitrate 500000 dbitrate 2000000 fd on

```
   

## To perform Forward Collision Warning System


## To perform Rear Collision Warning System
