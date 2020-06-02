# Fastron
A C++ implementation of Fastron based on the paper [Learning-Based Proxy Collision Detection for Robot Motion Planning Applications
](https://ieeexplore.ieee.org/abstract/document/9023003). If you use this work in your research, please cite

    @article{das2020learning,
      title={Learning-based Proxy Collision Detection for Robot Motion Planning Applications},
      author={Das, Nikhil and Yip, Michael},
      journal={IEEE Transactions on Robotics},
      year={2020},
      publisher={IEEE}
    }

## Installation
The contents of this repository can be placed in a directory to be used as a ROS package. For example, if `catkin_ws` is the ROS workspace, clone the repository into a subdirectory called `src/fastron`, and call `catkin_make` from `catkin_ws`. 

ROS launch files can be run as examples, e.g.,
```bash
roslaunch fastron baxter_fastron_collision_checking_test.launch
```
These example launch files are based on the Rethink Robotics Baxter robot, which will require the [Baxter model description files](https://github.com/RethinkRobotics/baxter_common) (`baxter_description`) and the [Baxter MoveIt! configuration files](https://github.com/ros-planning/moveit_robots) (`baxter_moveit_config`).

## Using Fastron
The main Fastron algorithm is in `fastron.cpp` and its header file `fastron.h`. Note that the kinematics collision detection (KCD) routine is a **virtual function** as it should be provided by the user. Thus, the `Fastron` class should not be used directly, but instead a derived class should be defined. The examples called by the launch files include such derived classes, where KCD routines are based on the FCL collision checking library.

The `Fastron` constructor only has a dataset of configurations (as an Eigen matrix of type double, where each row is a configuration) as an argument. Kernel and model parameters can be set after instantiation directly, e.g., `fastron.g = 10` can set the kernel parameter.

As defined in the paper, the pipeline of the Fastron algorithm involves three steps: updating the labels, updating the model, and active learning. These steps each have a method that should be called by the user, and they are described below.
```
              +----------------+    +---------------+
     data --->+ updateLabels() +--->+ updateModel() +---> model (eval())
              +----------------+    +---------------+
                ^      +------------------+       |
                +------+ activeLearning() +<------+
                       +------------------+
```
The `updateLabels()` method uses the user-defined KCD routine to update each label for each configuration in the dataset. If different collision detection methods are available, `updateLabels()` can take an integer argument to specify which method is used.

The `updateModel()` method updates the weights of the Fastron model.

The `activeLearning()` method augments the current dataset with new (unlabeled) configurations. This method should only be called after at least one instance of model updating.

The `eval()` method predicts the collision status label for a set of query configurations using the current Fastron model. The input is a pointer to an Eigen matrix of configurations, and the output is an Eigen array of doubles.

### Example
Assume that `FastronKCD` is a derived class from `Fastron` with a custom KCD routine.
```cpp
# Instantiate model
FastronKCD fastron(data); // data is an Eigen::MatrixXd where each row is a configuration

# Define model parameters
fastron.g = 10; // kernel parameter
fastron.beta = 100; // conditional bias parameter
fastron.maxSupportPoints = 3000; // support point cap
fastron.maxUpdates = 5000; // maximum number of update iterations
fastron.kNS = 2 * d; // number of points per support point for exploitation in active learning
fastron.sigma = 0.1; // standard deviation of Gaussians for exploitation sampling
fastron.allowance = 100; // size of active learning set

# Update the labels of the model
fastron.updateLabels();

# Update the model weights
fastron.updateModel();

# Predict values for a test set
fastron.eval(data_test);  // data_test is an Eigen::MatrixXd where each row is a query configuration
```

## Requirements
* Eigen
* ROS Kinetic
* MoveIt
* OMPL

## Credit
* Nikhil Das - [nkhldas](https://github.com/nkhldas)
