## 个人学习项目，参考Fast-LIO2，基于eigen库实现迭代右不变卡尔曼滤波（Iterated Right-Invariant Kalman Filter ）和迭代误差状态卡尔曼滤波（Iterated Error State Kalman Filter）的激光惯性里程计（开发中）
## Personal learning project, similar to Fast-LIO2, based on the eigen library to implement the laser inertial odometry of the Iterated Right-Invariant Kalman Filter (Iterated Right-Invariant Kalman Filter) and the Iterated Error State Kalman Filter (under development)


## Comparison Result :
Faster lio on m2dgr street 04
![alt text](images/fasterlio.png)
I-RIKF on m2dgr street 04
![alt text](images/ours.png)
map
![alt text](images/map.png)
pangolin
![alt text](images/pangolin.png)
## TODO

* [X] Add time_unit adaption for velodyne
* [X] Add trajectory and map save
* [X] Implement the iterated version of Right Invariant Kalman Filter
* [X] Add Pangolin,rid of stupid rviz (rviz still optinal though)
* [ ] Add gravity for invkf state
* [ ] Add IKdtree or Ivox to improve realtime performance (on the job)
* [X] Add loop clousre to improve global consitency (in branch dev/loop)
* [ ] Add extrinsic estimation

## Acknowledgments
 A big thanks to 
https://github.com/mengkai98/ieskf_slam for his detailed tutorial and excellent code，this project is built based on his work