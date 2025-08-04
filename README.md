# 个人学习项目，参考Fast-LIO2，基于eigen库实现多种滤波算法的激光惯性里程计（开发中）
## visualization
Faster lio on m2dgr street 04
![alt text](images/fasterlio.png)
I-RIKF on m2dgr street 04
![alt text](images/ours.png)
map
![alt text](images/map.png)
## TODO

* [X] Add time_unit adaption for velodyne
* [X] Add trajectory and map save
* [X] Implement the iterated version of Right Invariant Kalman Filter
* [ ] Add gravity for invkf state
* [ ] Add IKdtree or Ivox to improve realtime performance
* [ ] Add loop clousre to improve global consitency
* [ ] Add extrinsic estimation
