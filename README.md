



# Gesture-Control-of-Autonomous-Vehicle

In this repo, we develop a new interface of human interaction with Autonomous Vehicle.  

## Getting Started

These instructions will get you a copy of the project up and running on your local machine. You will need a fully functional autonomous vechile with camera configured to be able to run the control. But to simulate the control with print statement and run openpose, you don't really need a car.

### Prerequisites

What things you need to install the software and how to install them
* Python 2.7
* OpenCV 3.4
* ROS Kinetic
* Openpose(https://github.com/CMU-Perceptual-Computing-Lab/openpose)
* Openpose-ROS (https://github.com/ildoonet/ros-openpose)
```
Give examples
```

### Installing
Assuming you have ROS Kinetic in place
1. Install and compiling openpose following the instruction provided in the official github
2. Configure the paths in openpose-ros following the instruction provided in the above github
3. download and move the scripts and codes to the corresponding folder

## Running the tests

1. Run scripts/start_image_dd.sh to start a ros node that post downsampled camera feed(change the subscription topic to your hardware configuration)
2. Run scripts/start_openpose.sh to start openpose_ros, you should see a window popping up and pose being visualized
3. Run control_finetune.sh to start driving with hand gesture

Note: The system is developed assume extremely limited resources(GTX 1050 2GB), if you have better hardware you might want to use high resolution and frame rates.

### Gesture:
* Raise left hand to turn left
* Raise right hand to turn right
* Cross arm to accelerate
* Raise both hands to brake
```
Give an example
```

## Authors

* **Kaizhao Liang**
* **Dean Li**
* **Alex O'kennard**

## Demo
[![Demo1](https://i.imgur.com/vKb2F1B.png)](https://www.youtube.com/watch?v=nNd_gHcEi-4)
[![Demo2](https://i.imgur.com/vKb2F1B.png)](https://www.youtube.com/watch?v=5Q1_zXuTLAw)
## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* I want to thank my wonderful teammates Dean Li and Alex O'kennard for all the hardwork put in the project
* Also special acknowledgments for Dean's bravery to be the test subject
* I also to thank our instructor, David Forsyth for all his advices and insights
