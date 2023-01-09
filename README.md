# ros_midas

ROS Package for MidasV2.2 small.

### Node
The topic names can be changed in the param.yaml 
#### Subscribed Topics
1. /webcam/image_raw (sensor_msgs/Image)<br>
The input RGB image stream

#### Published Topics
1. /midas/depth_raw (sensor_msgs/Image) <br>
The resultant image after being passed through the detector. <br>

#### Services
1. start_depth (std_srv/SetBool) <br>
Switches on or off the estimator. 

### Requirements
Midas v2.1 small ([link](https://tfhub.dev/intel/lite-model/midas/v2_1_small/1/lite/1))
```
pip install numpy opencv-python tflite tensorflow
```


### Usage
`roslaunch ros_midas model.launch` <br>
`rosservice call /start_depth "data: false/true"`  to set the detection off/on
