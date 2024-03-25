% Mechatronics HW8 Problem C

% call moveTopDownCan
run('moveTopDownCan.m')
pause(2)

% close gripper around rCan3
grip_client = rosactionclient('/gripper_controller/follow_joint_trajectory',...
                              'control_msgs/FollowJointTrajectory', ...
                              'DataFormat', 'struct');
gripGoal = rosmessage(grip_client);
gripPos = 0.23;  % 0.8 is fully closed, 0 is fully open
gripGoal = packGripGoal(gripPos,gripGoal);
sendGoal(grip_client,gripGoal);

pause(4)

% print array
xyzrpy_array = [gripperX2 gripperY2 gripperZ2 0 0 pi/2];
fprintf('\nOutput:\n')
fprintf('[x y z r p y] = [%.4f %.4f %.4f %.4f %.4f %.4f]\n', xyzrpy_array)