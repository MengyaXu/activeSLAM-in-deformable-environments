============================
Active SLAM in deformable environment
============================

This is the code of the active deformable SLAM for the linear case and the nonlinear case. 

---------------
Quick start
---------------

For the both environments, run "EKF_main.m" to get the result of using a predetermined path or using the local planner.
Run "global_main.m" to get the result of using the global planner or the combined planner.

For both cases, the noises can be pre-generated by running "getNoise.m".
The motion of the features can be generated by running "getTrans.m".
For the global planner and combined planner, the viewpoints candidate can be obtained by running "grid.m".

--------------------------
Modes selection
--------------------------

Different models can be selected by changing the option parameter "op" at the beginning of the main files.

- "EKF_main.m"
    -- op = 1, the robot odometry is predetermined.
    -- op = 2, the robot odometry is planned by the local planner.
    
- "global_main.m"
    -- op = 21, the robot odometry is planned by the global planner.
    -- op = 22, the robot odometry is planned by the combined planner.

