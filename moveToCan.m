% Mechatronics HW8 Problem A

% close gripper
grip_client = rosactionclient('/gripper_controller/follow_joint_trajectory',...
                              'control_msgs/FollowJointTrajectory', ...
                              'DataFormat', 'struct');
gripGoal = rosmessage(grip_client);
gripPos = 0.8;  % 0.8 is fully closed, 0 is fully open
gripGoal = packGripGoal(gripPos,gripGoal);
sendGoal(grip_client,gripGoal);

% move arm
trajAct = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory', 'DataFormat','struct');

trajGoal = rosmessage(trajAct);

trajAct.FeedbackFcn = []; 

jointSub = rossubscriber("/joint_states");

jointStateMsg = jointSub.LatestMessage;

UR5e = loadrobot('universalUR5e', DataFormat="row");

tform=UR5e.Bodies{3}.Joint.JointToParentTransform;    
UR5e.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0]));
tform=UR5e.Bodies{4}.Joint.JointToParentTransform;
UR5e.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));
tform=UR5e.Bodies{7}.Joint.JointToParentTransform;
UR5e.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

ik = inverseKinematics("RigidBodyTree",UR5e); % Create Inverse kinematics solver

ikWeights = [0.25 0.25 0.25 0.1 0.1 .1]; % configuration weights for IK solver [Translation Orientation] see documentation

jointStateMsg = receive(jointSub,3); % receive current robot configuration

initialIKGuess = homeConfiguration(UR5e);

jointStateMsg.Name;

initialIKGuess(1) = jointStateMsg.Position(4); % update configuration in initial guess
initialIKGuess(2) = jointStateMsg.Position(3);
initialIKGuess(3) = jointStateMsg.Position(1);
initialIKGuess(4) = jointStateMsg.Position(5);
initialIKGuess(5) = jointStateMsg.Position(6);
initialIKGuess(6) = jointStateMsg.Position(7);

gripperX = -0.04;
gripperY = 0.8;
gripperZ = 0.19;

gripperTranslation = [gripperX gripperY gripperZ];
gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians


tform = eul2tform(gripperRotation); % ie eul2tr call
tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform

[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess);

UR5econfig = [configSoln(3)... 
              configSoln(2)...
              configSoln(1)...
              configSoln(4)...
              configSoln(5)...
              configSoln(6)];

trajGoal = packTrajGoal(UR5econfig,trajGoal);

sendGoal(trajAct,trajGoal);

pause(5)

% print array
xyzrpy_array = [gripperX gripperY gripperZ 0 0 pi/2];
fprintf('\nOutput:\n')
fprintf('[x y z r p y] = [%.4f %.4f %.4f %.4f %.4f %.4f]\n', xyzrpy_array)