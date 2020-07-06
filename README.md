# summit base test
## mobile_base_gazebo
```roslaunch mobile_base_gazebo spawn_summit_omni.launch``` use this line to launch the summit robot<br>
## mobile_base_navigation
`mobile_base_navigation` contains all the files for the robot to perform a multiple 
destination navigation.<br>
### create a new map
To perform an accurate navigation, a map need to be generated. you can typr the following command to launch a gmapping package.<br>
```roslaunch mobile_base_navigation start_mapping.launch```<br>
To save the map, just use map server:<br>
```rosrun map_server map_saver -f <name of the map>```<br>
Next, move two generated map file into the `maps` folder in the `mobile_base_description` package<br>
Then, modify the argument name of `map_file` on the first line of `start_localization.launch` in `launch` folder.<br>
### save destination point
To save the destination point, first you need to run the following script:<br>
```rosrun mobile_base_navigation click_save_spots.py <name of the file>```<br>
Secondly, you need to launch an rviz.<br>
```roslaunch mobile_base_navigation start_mapping.launch```<br>
In rviz, you can select the `publish point` tool in the toolbar, and then click on the map. Please pay attention not clicking on the obstacles, you can click on the surrounding area of the pieces you want to visit.<br>
Then, a csv file is created automatically in the spots folder.<br>
### reorganize the visit order to minimize the path length.
A genetic algorithm is used to optimize the total path length of visiting all the destinations. You can run the following script to generate a new reorgnized csv file.<br>
```rosrun mobile_base_navigation reorganize_pose.py <name of the file>```<br>
Then a new csv file will be generated and a plot shows the iteration of GA algorithm.<br>
### visit each point
To visit each point, you need to run the following script.<br>
```rosrun mobile_base_navigation visit_each_pose.py <name of the file>```<br>
A service is generated and you can visit the next destination by typing:<br>
```rosservice call /visit_next ```





