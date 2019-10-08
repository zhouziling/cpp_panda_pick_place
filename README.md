# Panda Pick and Place package

Package made to perform simple pick and place operations on colored cubes using a simulated Panda robot in Gazebo.

## Installation

### Install pre-requisites
* ROS melodic 
	* https://wiki.ros.org/melodic/Installation/Ubuntu
* MoveIt for melodic
	* https://moveit.ros.org/install/
* Franka gazebo simulation
	* https://github.com/erdalpekel/panda_simulation
* Jennifer Buehler's gazebo-pkgs
	* https://github.com/JenniferBuehler/gazebo-pkgs/wiki/Installation

### Install instructions
* Make sure you installed the melodic versions of ROS and MoveIt
* Clone "panda_simulation" package in your catkin directory with 
`git clone https://github.com/erdalpekel/panda_simulation.git`
and carefully follow its install instructions
* Clone "cpp_panda_pick_place" package in your catkin directory with 
`git clone https://github.com/iya-dt/cpp_panda_pick_place.git` 
and follow the instructions below 
    * The default parameters of the moveit planners are too strict to easily find feasible motions. Replacing `panda_moveit_config/launch/trajectory_execution.launch.xml` with `cpp_panda_pick_place/modif/trajectory_execution.launch.xml` fixes this. Otherwise, one would get "Controller is taking too long to execute trajectory". 
    * Gazebo physics engine is not optimized for grasping yet. The plugin provided in the package gazebo_grasp_plugin helps to overcome such issues : https://github.com/JenniferBuehler/gazebo-pkgs/wiki/Installation 
    Follow the package install instructions, then go in the `franka_description` package, replace `robots/panda_arm_hand.urdf.xacro` and `robots/panda_hand.xacro` with the files of the same name provided in `cpp_panda_pick_place/modif`. By doing so, we make the grasping possible in Gazebo.
    * catkin_make in your catkin directory :
	`catkin_make -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build`

<hr/>

## The demo
Once everything installed run the following commands in distinct terminals :
* `roslaunch panda_simulation simulation.launch load_grasp_fix:=true` and wait until simulation appears
* `roslaunch cpp_panda_pick_place spawn_objects.launch`  to spawn the objects in the scene

### pick and place service
once objects are spawned :
* launch the node with `rosrun cpp_panda_pick_place pick_and_place '{colour:"red_cube"}'`
* call the service with `rosservice call /pick_place_cube_service '{colour: "red_cube"}'`

### Reset the scene
If you'd like to reset the scene : 
* launch the node with `rosrun cpp_panda_pick_place reset_scene.py`
* call the service with `rosservice call /reset_scene_service`

<hr/>

## Notes

### Encountered issues & workarounds

* Issue in PlanningSceneInterface python wrapper (so I finally used C++ !) : 
	* they are incomplete : https://github.com/ros-planning/moveit/issues/1217
	* confusion between two different PlanningSceneInterface classes !	
* Gazebo tutorial from [Erdelpeka](https://github.com/erdalpekel/panda_simulation) : 
	* tutorials unclear : create a json repo ? `mkdir ~/.panda_simulation`
	* there doesn't seem to be a good grasp candidate generation in MoveIt, unless for cartesian planning there is Grasp Planner which will be eventually phased out in favor of the new MoveIt Task Constructor : https://github.com/ros-planning/moveit_task_constructor.
* Gazebo box grasping fails : 
	* this is because the physics engine is not optimized for grasping yet. The plugin provided in the package gazebo_grasp_plugin helps to overcome such issues : https://github.com/JenniferBuehler/gazebo-pkgs/wiki/Installation then panda_description has to be extended with an additional gazebo tag. 
	* other (dirty) solution : use `EffortJointInterface` instead, modifying panda_hand controllers locally. https://answers.ros.org/question/291228/simple-box-grasping-fails/
	* also see the fix under gripper friction in this link : https://buildmedia.readthedocs.org/media/pdf/de3-panda-wall/latest/de3-panda-wall.pdf
* Potential useful packages :
	* PICNIK (GraspIt) : https://github.com/ros-planning/moveit_grasps
	* EZGrasp : https://github.com/gstavrinos/ez_pick_and_place

### To improve
* documentation : file replacement instructions to be improved with shorter `cp`/`cd` commands
* file structure :
	* functions' documentation : make .hpp headers for the functions
* user friendliness :
	* create a launchFile : sim + spawn + node 
	* planners don't always find a valid path. Try to play with `setGoalTolerance`
* reset_scene_service : 
	* re-spawning cubes in random locations within the Panda workspace for more challenge.
* make it general :
	* Automatic real-time udpate of the planning scene based on Gazebo objects
* make it more realistic :
	* WARNING : Because the planners were too picky, only 1 cube at a time is included as an obstacle in the planningScene for the moment.
* make it optimal :
	* find a planning method with optimal trade-off between speed/accuracy/energy ? Or even better : let the user decide ;-)

### Language and package file structure convention
* https://wiki.ros.org/CppStyleGuide
* https://wiki.ros.org/Packages
* https://github.com/leggedrobotics/ros_best_practices/wiki