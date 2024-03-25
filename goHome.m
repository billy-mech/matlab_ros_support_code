trajAct = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory', 'DataFormat','struct') 

trajGoal = rosmessage(trajAct)

trajGoal = packTrajGoal([0,0,0,0,0,0],trajGoal)

sendGoal(trajAct,trajGoal)