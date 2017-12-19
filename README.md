#### Tools for CameraTrajectoryVisualization
* **Aim to implement it cross-platform**
* **Support “Trajectory” file Load and View**

    such as ORB-SLAM, DSO,OST-SLAM.
    their trajectory files have two classes:
    >**a:**
    >every line has 12 parameters,they compose one transform matrix:
    data[0],data[1],data[2],data[3],
    data[4],data[5],data[6],data[7],
    data[8],data[9],data[10],data[11]

    >**b:**
    >every line has 8 parameters, the first is line number,the rest is translation(3 paras) and quaternion (4 paras).
    quaternion includes the rotation information,we can compute the transform matrix using translation and quaternion:
    '''
             Pose(0,0) = 1 - 2*data[5]*data[5] - 2*data[6]*data[6];
             Pose(0,1) = 2*data[4]*data[5] + 2*data[3]*data[6];
             Pose(0,2) = 2*data[4]*data[6] - 2*data[3]*data[5];
             Pose(1,0) = 2*data[4]*data[5] - 2*data[3]*data[6];
             Pose(1,1) = 1 - 2*data[4]*data[4] - 2*data[6]*data[6];
             Pose(1,2) = 2*data[5]*data[6] + 2*data[3]*data[4];
             Pose(2,0) = 2*data[4]*data[6] + 2*data[3]*data[5];
             Pose(2,1) = 2*data[5]*data[6] - 2*data[3]*data[4];
             Pose(2,2) = 1 - 2*data[4]*data[4] - 2*data[5]*data[5];
             //T
             Pose(0,3) = data[0];
             Pose(1,3) = data[1];
             Pose(2,3) = data[2];
    '''
    change the view matrix and line color in Viewer.h if need


* **Dependence**:
```
 Pangolin, Eigen
```
* **Pangolin**

    Pangolin is a lightweight portable rapid development library for managing OpenGL display / interaction and abstracting video input.
[project on github](https://github.com/stevenlovegrove/Pangolin) 

* **Eigen**

    Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.
```
sudo apt-get install libeigen3-dev
```
