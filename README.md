# Panda Pick and Place package

Package made to perform simple pick and place operations on cubes using a simulated Panda robot in Gazebo.


### Installation pre-requisites
- ROS melodic
- MoveIt for melodic 
<!-- 
### MoveIt python bindings
https://wiki.ros.org/moveit_python
https://github.com/mikeferguson/moveit_python
sudo apt-get install ros-melodic-moveit-python
 -->
- Franka gazebo simulation
	- https://github.com/erdalpekel/panda_simulation
- Jennifer Buehler's gazebo-pkgs
	- https://github.com/JenniferBuehler/gazebo-pkgs/wiki/Installation

### Install instructions
- First make sure you have the melodic versions of ROS and MoveIt installed
<!-- here place install commands -->
- Clone the Gazebo Franka package in your catkin_ws and carefully follow install instructions
<!--  -->
- Clone the pick and place package in your catkin_ws and follow the instructions below
<!--  -->
	* The default planning parameters are too strict to easily find feasible motions.
Replacing `panda_moveit_config/launch/trajectory_execution.launch.xml` with `cpp_panda_pick_place/modif/trajectory_execution.launch.xml`
fixes this. Otherwise, one would get Controller is taking too long to execute trajectory 
	* Gazebo physics engine is not optimized for grasping yet. 
The plugin provided in the package gazebo_grasp_plugin helps to overcome such issues :
https://github.com/JenniferBuehler/gazebo-pkgs/wiki/Installation
Then panda_description has to be extended with an additional gazebo tag.
To do so, go in the `franka_description` package, replace `robots/panda_arm_hand.urdf.xacro` and `robots/panda_hand.xacro`
with the files of the same name provided in `cpp_panda_pick_place/modif/`
	* catkin_make (with libfranka options) in you catkin_ws :
	`catkin_make -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build`

### The demo
Once everything installed run the following commands in distinct terminals :
- `roslaunch panda_simulation simulation.launch load_grasp_fix:=true`
wait until simulation appears, then : 
- `roslaunch cpp_panda_pick_place spawn_objects.launch`

#### pick and place service
once objects are spawned :
- `rosrun cpp_panda_pick_place pick_and_place '{colour:"red_cube"}'`
then call the service :
- `rosservice call /pick_place_cube_service '{colour: "red_cube"}'`

#### Reset the scene
If you'd like to reset the scene : 
- `rosrun cpp_panda_pick_place reset_scene.py`
then call the service :
- `rosservice call /reset_scene_service`

### Occasional issues :
1. Gazebo box grasping fails : 
- this is because the physics engine is not optimized for grasping yet. The plugin provided in the package gazebo_grasp_plugin helps to overcome such issues :
https://github.com/JenniferBuehler/gazebo-pkgs/wiki/Installation
then panda_description has to be extended with an additional gazebo tag.
- other (dirty) solution : use EffortJointInterface instead, modifying panda_hand controllers locally.
https://answers.ros.org/question/291228/simple-box-grasping-fails/
- also see the fix under gripper friction in this link :
https://buildmedia.readthedocs.org/media/pdf/de3-panda-wall/latest/de3-panda-wall.pdf

### Notes : 
- issue in PlanningSceneInterface python wrapper : 
	* they are incomplete : https://github.com/ros-planning/moveit/issues/1217
	* confusion between two different PlanningSceneInterface classes !	
- Gazebo from Erdelpeka : 
	* tutorials unclear : create a json repo ? `mkdir ~/.panda_simulation`
	* there doesn't seem to be a good grasp candidate generation in MoveIt, unless for cartesian planning there is Grasp Planner 
	which will be eventually phased out in favor of the new MoveIt Task Constructor : https://github.com/ros-planning/moveit_task_constructor.
- Potential Improvements :
	* PICNIK (GraspIt) : https://github.com/ros-planning/moveit_grasps
	* EZGrasp : https://github.com/gstavrinos/ez_pick_and_place

### TODO :
- file structure :
	* finalize documentation : make .hpp headers for the functions
- user friendliness :
	* create a launchFile : sim + spawn + node 
	* planners don't always find a valid path. Try to play with `setGoalTolerance (double tolerance)`
- reset_scene_service : 
	* re-spawning cubes in random locations within the Panda workspace for more challenge.
- make it general :
	* Automatic real-time udpate of the planning scene based on Gazebo objects
- make it more realistic :
	* WARNING : Because the planners were too picky, only 1 cube at a time is included as an obstacle in the planningScene for the moment.
- make it optimal :
	* find a planning method with optimal trade-off between speed/accuracy/energy ? Or even better : let the user decide ;-)

### Language and package file structure convention : 
- https://wiki.ros.org/CppStyleGuide
- https://wiki.ros.org/Packages
- https://github.com/leggedrobotics/ros_best_practices/wiki