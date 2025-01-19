# Grasp Detection algorithms

Directed Research under Prof Berk Calli (Worcester Polytechnic Institute)

## Installation Instructions
```
pip install torch easydict tensorflow tf_slim shapely
pip install --upgrade opencv-python
mkdir -p grasp_algo_ws/src
cd grasp_algo_ws/src
git clone https://github.com/cdbharath/grasp_synthesis
cd ..
catkin build
source devel/setup.bash
```

## How to run:
```
# To run Mask Based Algo
rosrun mask_based_algo service_server.py

# To run Top Surface Algo
roslaunch top_surface_algo top_surface.launch

# To run GGCNN
rosrun ggcnn service_server.py

# To run ResNet Based Algo
rosrun ros_deep_grasp service_server.py

```


## References:
1. [GGCNN ROS wrapper](https://github.com/dougsm/mvp_grasp "GGCNN ROS wrapper")
2. [Learning robust, real-time reactive robotic grasping](https://journals.sagepub.com/doi/full/10.1177/0278364919859066 "Learning robust, real-time rective robotic grasping") 
3. [Multi Object Multi Grasp Predictor](https://github.com/ivaROS/ros_deep_grasp "Multi Object Multi Grasp Predictor") 

