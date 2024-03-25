% Mechatronics HW8 Problem B

% move gripper horizontally above rCan3
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
initialIKGuess(4) = jointStateMsg.Position(5) - 0.5;
initialIKGuess(5) = jointStateMsg.Position(6);
initialIKGuess(6) = jointStateMsg.Position(7);

gripperX = -0.04;
gripperY = 0.75;
gripperZ = 0.5;

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

% move arm elbow up to rCan3
trajAct2 = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory', 'DataFormat','struct');

trajGoal2 = rosmessage(trajAct2);

trajAct2.FeedbackFcn = []; 

jointSub2 = rossubscriber("/joint_states");

jointStateMsg2 = jointSub2.LatestMessage;

UR5e = loadrobot('universalUR5e', DataFormat="row");

tform2=UR5e.Bodies{3}.Joint.JointToParentTransform;    
UR5e.Bodies{3}.Joint.setFixedTransform(tform2*eul2tform([pi/2,0,0]));
tform2=UR5e.Bodies{4}.Joint.JointToParentTransform;
UR5e.Bodies{4}.Joint.setFixedTransform(tform2*eul2tform([-pi/2,0,0]));
tform2=UR5e.Bodies{7}.Joint.JointToParentTransform;
UR5e.Bodies{7}.Joint.setFixedTransform(tform2*eul2tform([-pi/2,0,0]));

ik2 = inverseKinematics("RigidBodyTree",UR5e); % Create Inverse kinematics solver

ikWeights2 = [0.25 0.25 0.25 0.1 0.1 .1]; % configuration weights for IK solver [Translation Orientation] see documentation

jointStateMsg2 = receive(jointSub2,3); % receive current robot configuration

initialIKGuess2 = homeConfiguration(UR5e);

jointStateMsg2.Name;

initialIKGuess2(1) = jointStateMsg2.Position(4); % update configuration in initial guess
initialIKGuess2(2) = jointStateMsg2.Position(3);
initialIKGuess2(3) = jointStateMsg2.Position(1);
initialIKGuess2(4) = jointStateMsg2.Position(5) - 0.5;
initialIKGuess2(5) = jointStateMsg2.Position(6);
initialIKGuess2(6) = jointStateMsg2.Position(7);

gripperX2 = -0.04;
gripperY2 = 0.8;
gripperZ2 = 0.14;

gripperTranslation2 = [gripperX2 gripperY2 gripperZ2];
gripperRotation2 = [-pi/2 -pi 0]; %  [Z Y X]radians


tform2 = eul2tform(gripperRotation2); % ie eul2tr call
tform2(1:3,4) = gripperTranslation2'; % set translation in homogeneous transform

[configSoln2, solnInfo2] = ik2('tool0',tform2,ikWeights2,initialIKGuess2);

UR5econfig2 = [configSoln2(3)... 
              configSoln2(2)...
              configSoln2(1)...
              configSoln2(4)...
              configSoln2(5)...
              configSoln2(6)];

trajGoal2 = packTrajGoal(UR5econfig2,trajGoal2);

sendGoal(trajAct2,trajGoal2);

pause(5)

% print array
xyzrpy_array = [gripperX2 gripperY2 gripperZ2 0 0 pi/2];
fprintf('\nOutput:\n')
fprintf('[x y z r p y] = [%.4f %.4f %.4f %.4f %.4f %.4f]\n', xyzrpy_array)
