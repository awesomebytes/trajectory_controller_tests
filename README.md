trajectory_controller_tests
===========================

Just some random python tests interfacing with trajectory controllers


====

Probably the most interesting one is ```scripts/controller_crazier.py```.

It reads all the controllers available in the controller manager (using the service
/controller_manager/list_controllers), sets up publishers for their topic command interfaces,
reads the joint limits values of each joint and sends random goals between those values to all
joints of all groups in with random different timings.

Why? Just for fun. Have a look at the results with REEM-C:

https://www.youtube.com/watch?v=UuHJQLcnpDo

====

You can try it by yourself installing REEM-C simulation following this instructions:

http://wiki.ros.org/Robots/REEM-C/Tutorials/launch

Then you can just launch the simulation and the joint controllers:

    roslaunch reemc_gazebo reemc_empty_world.launch
    roslaunch reemc_controller_configuration joint_trajectory_controllers.launch

And, after it loads, you can just launch the node:

	rosrun trajectory_controller_tests controller_crazier.py

