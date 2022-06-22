function jog(changeInPosition, axis, robot)

global estop;
steps=10;
% Discrete time step
deltaT = 0.01;                                                              
minManipMeasure = 0.1;
deltaTheta = 2*pi/steps;
if estop==0
    % finding current and desired joint angles
    q = robot.model.getpos;
    tr = robot.model.fkine(q);
    tr(axis,4) = tr(axis,4) + changeInPosition;
    newQ = robot.model.ikcon(tr,q);
    
    % to make the points, first use jtraj
    jointsOfPoints=jtraj(q, newQ, steps);
    % From jtraj, convert joint values to end effector coordinates:
    % (Creating trajectory, where x is a matrix for the x and y coordinates?)
    for i = 1:steps
        temp = robot.model.fkine(jointsOfPoints(i,:));
        x(:,i) = [temp(1,4);temp(2,4);temp(3,4)];
        disp(x(:,i));
    end

    %initial qMatrix
    qMatrix(1,:)=q;                    
    % Matrix for Measure of Manipulability
    m = zeros(1,steps);

    % Use Resolved Motion Rate Control to solve joint velocities at each time step
    % that will make the end-effector follow the Cartesian trajectory created.
    for i = 1:steps-1
    
        % Calculate velocity at discrete time step, by analysing the change in
        % cartesian coordinates divided by time step
        xdot = (x(:,i+1) - x(:,i))/deltaT;                                      

        % Calculate Jacobian for the specific time step
        J = robot.model.jacob0(qMatrix(i,:));                                            
        J = J(1:3,:);

        % Measure of Manipulability
        m(:,i)= sqrt(det(J*J'));
    
        if m(:,i) < minManipMeasure
            qdot = inv(J'*J + 0.01*eye(6))*J'*xdot;
        else
            % Solve velocities via RMRC
            qdot = inv(J) * xdot; 

        end
        % Update next joint state
        qMatrix(i+1,:) = qMatrix(i,:) + deltaT * qdot';                         
    end

    robot.model.animate(qMatrix);
    
else
    display('ESTOP IN ON');
end

updateJoint(robot);