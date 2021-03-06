%% global variables/ variables
dimension=2.5;
steps=20;
offsetFromBase=transl(1.3,0,0.77);

%% TM5 robot setup
r=TM5;
r.model.teach;
q = zeros(1,6);
r.model.base=r.model.base*offsetFromBase;
view(3);
hold on;
r.model.animate(q);

%% set up environment
% background setup
axis equal
% % camlight
% surf([-dimension,-dimension;dimension,dimension],[-dimension,dimension;-dimension,dimension],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
% surf([-dimension,dimension;-dimension,dimension],[-dimension,-dimension;-dimension,-dimension],[-0,-0;dimension,dimension],'CData',flipdim(flip(imread('mountainTemple.jpg')),2),'FaceColor','texturemap');
% surf([-dimension,-dimension;-dimension,-dimension],[-dimension,-dimension;dimension,dimension],[-0,dimension;-0,dimension],'CData',imrotate(imread('mountainTopView.jpg'),-90,'bilinear'),'FaceColor','texturemap');

%table setup
PlaceObject('pingPongTable.ply', [0 0 0.8]); 

% % ping pong racket and ball setup
% B1=PlaceObject('pingPongPaddle.ply', [0 0 0]);B1_vertices=get(B1,'Vertices');
% tr=r.model.fkine(r.model.getpos())*transl(0,0,0.02);
% transformedVertices=[B1_vertices,ones(size(B1_vertices,1),1)]*tr';
% set(B1,'Vertices',transformedVertices(:,1:3));
% 
% % new one: ball is at fixed location, so robot will hit it 
% B2=PlaceObject('pingPongBall.ply', [0 0 0]);B2_vertices=get(B2,'Vertices');
% balltr=transl(0.8,0,1.4);
% transformedVertices=[B2_vertices,ones(size(B2_vertices,1),1)]*balltr';
% set(B2,'Vertices',transformedVertices(:,1:3));
% 
% %safety infrastructure
% origin=transl(0,0,0);
% y1=0.75;
% y2=-y1;
% 
% Cone=origin*transl([-1, 2*y1, 0]);
% PlaceObject('cone.ply', [Cone(1:3,4)']);
% hold on;
% Cone=origin*transl([-0.5, 2*y1, 0]);
% PlaceObject('cone.ply', [Cone(1:3,4)']);
% Cone=origin*transl([0, 2*y1, 0]);
% PlaceObject('cone.ply', [Cone(1:3,4)']);
% Cone=origin*transl([0.5, 2*y1, 0]);
% PlaceObject('cone.ply', [Cone(1:3,4)']);
% Cone=origin*transl([1, 2*y1, 0]);
% PlaceObject('cone.ply', [Cone(1:3,4)']);
% %cones on other side of robots:
% Cone=origin*transl([-1, 2*y2, 0]);
% PlaceObject('cone.ply', [Cone(1:3,4)']);
% Cone=origin*transl([-0.5, 2*y2, 0]);
% PlaceObject('cone.ply', [Cone(1:3,4)']);
% Cone=origin*transl([0, 2*y2, 0]);
% PlaceObject('cone.ply', [Cone(1:3,4)']);
% Cone=origin*transl([0.5, 2*y2, 0]);
% PlaceObject('cone.ply', [Cone(1:3,4)']);
% Cone=origin*transl([1, 2*y2, 0]);
% PlaceObject('cone.ply', [Cone(1:3,4)']);

%%
steps=100;
% Discrete time step
deltaT = 0.01;                                                              
minManipMeasure = 0.1;
deltaTheta = 2*pi/steps;

q = r.model.getpos;
tr = r.model.fkine(q);
tr(1,4) = tr(1,4) + 0.01;
newQ = r.model.ikcon(tr,q);
qMatrix(1,:)=q;
r.model.jtraj(q, newQ, steps);

    % From jtraj, convert joint values to end effector coordinates:
    % (Creating trajectory, where x is a matrix for the x and y coordinates?)
    for i = 1:steps
        temp = r.model.fkine(q(i,:));
        temp=temp(1:3,4);
        x(:,i)=temp(:,i);
    end
    % Matrix for Measure of Manipulability
    m = zeros(1,steps);
    
    % Use Resolved Motion Rate Control to solve joint velocities at each time step
    % that will make the end-effector follow the Cartesian trajectory created.
    for i = 1:steps-1
        % Calculate velocity at discrete time step, by analysing the change in
        % cartesian coordinates divided by time step
        xdot = (x(:,i+1) - x(:,i))/deltaT;                                      
       
        % Calculate Jacobian for the specific time step
        J = r.model.jacob0(qMatrix(i,:));                                            
        J = J(1:6,:);
        % Measure of Manipulability
        m(:,i)= sqrt(det(J*J'));
    
        if m(:,i) < minManipMeasure
            qdot = inv(J'*J + 0.01*eye(2))*J'*xdot;
        else
            % Solve velocities via RMRC
            qdot = inv(J) * xdot;                                               
        end
        % Update next joint state
        qMatrix(i+1,:) = qMatrix(i,:) + deltaT * qdot';                         
    end
    r.model.animate(qMatrix);



%% Programming robot
% use reverse kinematics so that z axis of end effector faces ping pong ball parallel to the
% x axis (global), meaning it is preparing for straight shot.
newTr=balltr*transl(1,0,-0.07);
trAngle=troty(deg2rad(-90))*trotz(deg2rad(180));
newTr(1:3,1:3)=trAngle(1:3,1:3);

qMatrix=jtraj(r.model.getpos(),r.model.ikcon(newTr, r.model.getpos()),steps);
for i=1:steps
    r.model.animate(qMatrix(i,:));
    tr=r.model.fkine(r.model.getpos())*transl(0,0,0.02);
    transformedVertices=[B1_vertices,ones(size(B1_vertices,1),1)]*tr';
    set(B1,'Vertices',transformedVertices(:,1:3));
end
% 
% %%
%0.5m away, and travelling at 1m/s:
t = 4;             % Total time (s)
deltaT = 0.02;      % Control frequency
steps = t/deltaT;   % No. of steps for simulation
delta = 2*pi/steps; % Small angle change
epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares

% 1.3) Set up trajectory, initial pose
qMatrix = zeros(steps,6);       % Array for joint anglesR
theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
x = zeros(3,steps);             % Array for x-y-z trajectory

s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
tr=r.model.fkine(r.model.getpos())*transl(0,0,0);
rpy=tr2rpy(r.model.fkine(r.model.getpos()))
    x(1,1) = tr(1,4); % Points in x
    x(2,1) = tr(2,4); % Points in y
    x(3,1) = tr(3,4); % Points in z
    theta(1,1) = rpy(1);                 % Roll angle 
    theta(2,1) = rpy(2);                 % Pitch angle
    theta(3,1) = rpy(3);                 % Yaw angle
for i=2:steps
    x(1,i) = x(1,i-1)-0.005; % Points in x
    x(2,i) = x(2,1); % Points in y
    x(3,i) = x(3,1); % Points in z
    theta(1,i) = theta(1,1);                 % Roll angle 
    theta(2,i) = theta(2,1);            % Pitch angle
    theta(3,i) = theta(3,1);                 % Yaw angle
end
 
T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
q0 = zeros(1,6);                                                            % Initial guess for joint angles
qMatrix(1,:) = r.model.ikcon(T,q0);                                            % Solve joint angles to achieve first waypoint

%1.4) Track the trajectory with RMRC
m = zeros(steps,1);             % Array for Measure of Manipulability
qdot = zeros(steps,6);          % Array for joint velocities
positionError = zeros(3,steps); % For plotting trajectory error
angleError = zeros(3,steps);    % For plotting trajectory error

for i = 1:steps-1
    T = r.model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint. This is important as forward kinematics often has an error

    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error, and times by inverse delta t for velocity

    S = Rdot*Ra';                                                           % Skew symmetric!
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
    deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles

    W = diag([1 1 1 0.1 0.1 0.1]);                                          % Weighting matrix for the velocity vector. This case means more influence to linear velocity than angular velocity
    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    
    J = r.model.jacob0(qMatrix(i,:));                                          % Get Jacobian at current joint state
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon                                                       % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the         vector)
    for j = 1:6                                                             % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < r.model.qlim(j,1)                 % If next joint angle is lower than joint limit...
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > r.model.qlim(j,2)             % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                      	% Update next joint state based on joint velocities
    positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
    angleError(:,i) = deltaTheta;                                           % For plotting
end
% 1.5) Plot the results
for i=1:steps
    r.model.animate(qMatrix(i,:));
    tr=r.model.fkine(r.model.getpos())*transl(0,0,0.02);
    transformedVertices=[B1_vertices,ones(size(B1_vertices,1),1)]*tr';
    set(B1,'Vertices',transformedVertices(:,1:3));
end

%% Visual Servoing (placing paddle down)
% Create image target (points in the image plane) 
pStar = [662 362 362 662; 362 362 662 662];
%Create 3D points
P=[1.3,1.3,1.2,1.2;
-0.45 ,-0.35,-0.35,-0.45;
0.8,0.8,0.8,0.8];
q0 = [deg2rad(50); deg2rad(20); deg2rad(40); deg2rad(-70); deg2rad(-80); deg2rad(-45)];

% Add the camera
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5,'resolution', [1024 1024], 'centre', [512 512],'name', 'UR10camera');
fps = 25;% frame rate
%Define values
lambda = 0.6; %gain of the controler
depth = mean (P(3,:)); %depth of the IBVS
% 1.2 Initialise Simulation (Display in 3D)
Tc0= r.model.fkine(q0);
r.model.animate(q0');
drawnow
% Display points in 3D and the camera
cam.plot_camera('Tcam',Tc0, 'label','scale',0.15);
plot_sphere(P, 0.02, 'r')
light
% 1.3 Initialise Simulation (Display in Image view)
%Project points to the image
cam.plot(pStar, '*'); % create the camera view
cam.hold(true);
cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
%Initialise display arrays
vel_p = [];
uv_p = [];
Zest = []; %stands for Z estimated, as we roughly assume the target's depth

%1.4 Loop of the visual servoing
ksteps = 0;
maxSteps=350;
 while true
        ksteps = ksteps + 1;
        
        % compute the view of the camera
        uv = cam.plot(P);
        % compute image plane error as a column
        e = pStar-uv;   % feature error
        e = e(:);       % e (error) is 2x4 matrix. This is converted into an 8by1 matrix AKA vector
        
        % compute the Jacobian
        if isempty(depth)
            % exact depth from simulation (not possible in practice)
            pt = homtrans(inv(Tcam), P); % WHAT IS HAPPENING HERE: homtrans(T, p) applies homogeneous transformation T to the points stored columnwise in p. 
            J = cam.visjac_p(uv, pt(3,:) ); % image jacobian for the image plane
        elseif ~isempty(Zest)
            J = cam.visjac_p(uv, Zest);
        else
            J = cam.visjac_p(uv, depth );
        end

        % compute the velocity of camera in camera frame
        try
            v = lambda * pinv(J) * e; % Compute the desired velocity ofthe camera given the error andthe Image Jacobian
        catch
            status = -1;
            return
        end
        fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);

        %compute robot's Jacobian and inverse
        J2 = r.model.jacobn(q0);
        Jinv = pinv(J2);
        % get joint velocities
        qp = Jinv*v;

         %Maximum angular velocity cannot exceed 180 degrees/s
         ind=find(qp>pi);
         if ~isempty(ind)
             qp(ind)=pi;
         end
         ind=find(qp<-pi);
         if ~isempty(ind)
             qp(ind)=-pi;
         end

        %Update joints 
        q = q0 + (1/fps)*qp;        

        for i=1:steps
        r.model.animate(q');
        tr=r.model.fkine(r.model.getpos())*transl(0,0,0.02);
        transformedVertices=[B1_vertices,ones(size(B1_vertices,1),1)]*tr';
        set(B1,'Vertices',transformedVertices(:,1:3));
        end

        %Get camera location
        Tc = r.model.fkine(q);
        cam.T = Tc;
        drawnow
        if ~isempty(maxSteps) && (ksteps > maxSteps)
            break;
        end
        %update current joint position
        q0 = q;
 end