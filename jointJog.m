function jointJog(jointNumber, robot, hObject)

global estop;

if estop==0
    j=get(hObject,'Value');
    concat=strcat('j',int2str(jointNumber),'Text');
    set(robot.concat);
    q=robot.model.getpos();
    q(jointNumber)=deg2rad(j);
    tr = robot.model.fkine(q);
    robot.model.animate(q);
else
    display('ESTOP IN ON');
end

updateJoint(robot);