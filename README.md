# Aruco_detection

## To run A* node:

roslaunch find_path Astar_navigation.launch

Once the Rviz starts you can provide the start point with 2D pose estimate and end point with 2D navgoal. You can visualize the generated path.
Note:Please make sure both the points are inside the map.
Feel free to try any other maze maps if you could.

## To run Aruco detection:

First make sure the camera topic is published '/camera/color/image_raw'.

once it is done you can run the action server by,

rosrun detection_pkg detection_pkg_node

Then start the client,

rosrun detection_pkg action_client_node

to see the x,y,theta values check,

rostopic echo /marker_detection/result 
