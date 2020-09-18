


README
=============
![ezgif com-video-to-gif](https://user-images.githubusercontent.com/67350632/93546526-e7102c80-f99d-11ea-8179-081bdf64e6d5.gif)

Bounding Box, ID 시각화
-------------------------

### master node 실행

  roscore
### Bag 파일 실행

  rosbag play -l [File_Name.bag]

### RVIZ 실행

  rviz
- Fixed Frame : velo_link
- PointCloud2 : /kitti/velo/pointcloud


### rqt 그래프 실행

  rqt_graph

### Tracklet 패키지 실행

  rosrun tracklet_pkg bounding_pub.py

### RVIZ 노드 추가

- BoundingBoxArray : Topic - /kitti_box
