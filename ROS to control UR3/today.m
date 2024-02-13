rosinit('192.168.0.253'); % Assuming a UTS Pi, otherwise please change this
%%
jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
% Pause to give time for a message to appear
pause(2);
% retrieve current joint states (note in weird order)
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
% reorganise joint states
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
% Create variables with joint names so that the joint commands are associated with a particular joint
jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

%% Rotating the first (shoulder_pan) and last (wrist_3) joints by pi/8 in 5 seconds

% To send a set of joint angles to the robot, you need to create a 'client' and define a 'goal'
% The function rosactionclient (Links to an external site.) will create these variables, and allow
% you to use them to "connect to an action server using a SimpleActionClient object and request
% the execution of action goals."

[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);
 % This allows for the time taken to send the message. If the network is fast, this could be reduced.
bufferSeconds = 1;
% This is how many seconds the movement will take
durationSeconds = 5;

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
% sending the robot to the current joint state
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
% sending the robot to an offset of the current joint state
nextJointState_123456 = currentJointState_123456 + [pi/4,0,0,0,0,pi/2];
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

% creating trajectory for the robot to perform`
goal.Trajectory.Points = [startJointSend; endJointSend];
%%
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);