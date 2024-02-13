clear all;
close all;

jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
pause(2); % Pause to give time for a message to appear


dimension=0.5;
L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',[-pi pi]);
L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim',[-pi pi]);
L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim',[-pi pi]);
L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',[-pi pi]);
L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',[-pi pi]);
L6 = Link('d',0.0819,'a',0,'alpha',0,'qlim',[-pi pi]);

robot = SerialLink([L1 L2 L3 L4, L5, L6],'name','UR3');
q = zeros(1,6);

qq=retrieve_q_value(jointStateSubscriber)
robot.plot(qq,'workspace',[-dimension dimension -dimension dimension 0 dimension],'scale',0.1);
robot.teach;

hold on;

steps=100
q=robot.ikcon(transl(0.3,0.3,0.1)*trotx(pi), robot.getpos())
matrix=jtraj(robot.getpos(),q,steps)

for i=1:steps
    robot.animate(matrix(i,:))
end
assign_q_value(jointStateSubscriber,q,10)


q1=robot.ikcon(transl(0.3,0.4,0.5)*trotx(pi), robot.getpos())
matrix=jtraj(robot.getpos(),q1,steps)

for i=1:steps
    robot.animate(matrix(i,:))
end
assign_q_value(jointStateSubscriber,q1,10)



q2=robot.ikcon(transl(-0.3,-0.3,0.3)*trotx(pi), robot.getpos())
matrix=jtraj(robot.getpos(),q2,steps)

for i=1:steps
    robot.animate(matrix(i,:))
end
assign_q_value(jointStateSubscriber,q2,10)

