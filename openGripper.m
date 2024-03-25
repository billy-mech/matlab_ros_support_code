% open gripper
grip_client = rosactionclient('/gripper_controller/follow_joint_trajectory',...
                              'control_msgs/FollowJointTrajectory', ...
                              'DataFormat', 'struct')
gripGoal = rosmessage(grip_client)
gripPos = 0.0  % 0.8 is fully closed, 0 is fully open
gripGoal = packGripGoal(gripPos,gripGoal)
sendGoal(grip_client,gripGoal)