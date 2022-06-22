function updateJoint(robot)

    tr=robot.model.fkine(robot.model.getpos())
    set(robot.poseTX,'String',tr(1,4))
    set(robot.poseTY,'String',tr(2,4))
    set(robot.poseTZ,'String',tr(3,4))
    
    rpy=tr2rpy(robot.model.fkine(robot.model.getpos()))
    set(robot.poseRX,'String',rpy(1))
    set(robot.poseRY,'String',rpy(2))
    set(robot.poseRZ,'String',rpy(3))

end