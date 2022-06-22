function varargout = untitledGUI(varargin)
% UNTITLEDGUI MATLAB code for untitledGUI.fig
%      UNTITLEDGUI, by itself, creates a new UNTITLEDGUI or raises the existing
%      singleton*.
%
%      H = UNTITLEDGUI returns the handle to a new UNTITLEDGUI or the handle to
%      the existing singleton*.
%
%      UNTITLEDGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in UNTITLEDGUI.M with the given input arguments.
%
%      UNTITLEDGUI('Property','Value',...) creates a new UNTITLEDGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before untitledGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to untitledGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help untitledGUI

% Last Modified by GUIDE v2.5 30-May-2022 14:58:35

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @untitledGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @untitledGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before untitledGUI is made visible.
function untitledGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.

% Choose default command line output for untitledGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% This sets up the initial plot - only do when we are invisible
% so window can get raised using untitledGUI.
if strcmp(get(hObject,'Visible'),'off')
    plot(rand(5));
end

% UIWAIT makes untitledGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = untitledGUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
cla
axes(handles.axes1);

L1 = Link('d',0.1452,'a',0,'alpha',pi/2,'qlim',[-deg2rad(270) deg2rad(270)]);
L2 = Link('d',0,'a',-0.329,'alpha',0,'qlim',[-pi pi], 'offset', -pi/2);
L3 = Link('d',0,'a',-0.311,'alpha',0,'qlim',[-deg2rad(155) deg2rad(155)]);
L4 = Link('d',0.106,'a',0,'alpha',pi/2,'qlim',[-pi pi]);
L5 = Link('d',0.106,'a',0,'alpha',-pi/2,'qlim',[-pi pi]);
L6 = Link('d',0.1132,'a',0,'alpha',0,'qlim',[-deg2rad(270) deg2rad(270)]);

model = SerialLink([L1 L2 L3 L4 L5 L6],'name','TM5');

for linkIndex = 0:model.n
    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['TM5Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>        
    model.faces{linkIndex+1} = faceData;
    model.points{linkIndex+1} = vertexData;
end

% Display robot
workspace = [-2 2 -2 2 -0.3 2];   
model.plot3d(zeros(1,model.n),'noarrow','workspace',workspace);
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end  
model.delay = 0;

% Try to correctly colour the arm (if colours are in ply file data)
for linkIndex = 0:model.n
    handles = findobj('Tag', model.name);
    h = get(handles,'UserData');
    try 
        h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                      , plyData{linkIndex+1}.vertex.green ...
                                                      , plyData{linkIndex+1}.vertex.blue]/255;
        h.link(linkIndex+1).Children.FaceColor = 'interp';
    catch ME_1
        disp(ME_1);
        continue;
    end
end
    
data = guidata(hObject);
data.model = model;
guidata(hObject,data);

dimension=2;
hold on;
surf([-dimension,-dimension;dimension,dimension],[-dimension,dimension;-dimension,dimension],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
surf([-dimension,dimension;-dimension,dimension],[-dimension,-dimension;-dimension,-dimension],[-0,-0;dimension,dimension],'CData',flipdim(flip(imread('mountainTemple.jpg')),2),'FaceColor','texturemap');
surf([-dimension,-dimension;-dimension,-dimension],[-dimension,-dimension;dimension,dimension],[-0,dimension;-0,dimension],'CData',imrotate(imread('mountainTopView.jpg'),-90,'bilinear'),'FaceColor','texturemap');

%safety infrastructure
origin=transl(0,0,0);
y1=0.75;
y2=-y1;

Cone=origin*transl([-1, 2*y1, 0]);
PlaceObject('cone.ply', [Cone(1:3,4)']);
hold on;
Cone=origin*transl([-0.5, 2*y1, 0]);
PlaceObject('cone.ply', [Cone(1:3,4)']);
Cone=origin*transl([0, 2*y1, 0]);
PlaceObject('cone.ply', [Cone(1:3,4)']);
Cone=origin*transl([0.5, 2*y1, 0]);
PlaceObject('cone.ply', [Cone(1:3,4)']);
Cone=origin*transl([1, 2*y1, 0]);
PlaceObject('cone.ply', [Cone(1:3,4)']);
%cones on other side of robots:
Cone=origin*transl([-1, 2*y2, 0]);
PlaceObject('cone.ply', [Cone(1:3,4)']);
Cone=origin*transl([-0.5, 2*y2, 0]);
PlaceObject('cone.ply', [Cone(1:3,4)']);
Cone=origin*transl([0, 2*y2, 0]);
PlaceObject('cone.ply', [Cone(1:3,4)']);
Cone=origin*transl([0.5, 2*y2, 0]);
PlaceObject('cone.ply', [Cone(1:3,4)']);
Cone=origin*transl([1, 2*y2, 0]);
PlaceObject('cone.ply', [Cone(1:3,4)']);

% --- Executes on button press in setup.
function setup_Callback(hObject, eventdata, handles)
global balltr;
global tr;
global B1_vertices;
global B2_vertices;
global B1;
global B2;

axis equal
camlight
%table setup
PlaceObject('pingPongTable.ply', [0 0 0.8]); 

% ping pong racket and ball setup
B1=PlaceObject('pingPongPaddle.ply', [0 0 0]);B1_vertices=get(B1,'Vertices');
tr=handles.model.fkine(handles.model.getpos())*transl(0,0,0.02);
transformedVertices=[B1_vertices,ones(size(B1_vertices,1),1)]*tr';
set(B1,'Vertices',transformedVertices(:,1:3));

% new one: ball is at fixed location, so robot will hit it 
B2=PlaceObject('pingPongBall.ply', [0 0 0]);B2_vertices=get(B2,'Vertices');
balltr=transl(0.8,0,1.4);
transformedVertices=[B2_vertices,ones(size(B2_vertices,1),1)]*balltr';
set(B2,'Vertices',transformedVertices(:,1:3));

offsetFromBase=transl(1.3,0,0.77);
q = zeros(1,6);
handles.model.base=handles.model.base*offsetFromBase;
view(3);
handles.model.animate(q);

% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end
delete(handles.figure1)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
     set(hObject,'BackgroundColor','white');
end
set(hObject, 'String', {'plot(rand(5))', 'plot(sin(1:0.01:25))', 'bar(1:.5:10)', 'plot(membrane)', 'surf(peaks)'});

% with singularity protection:
function plusX_pushbutton_Callback(hObject, eventdata, handles)
jog(+0.01, 1, handles);

% --- Executes on button press in minusX_pushbutton.
function minusX_pushbutton_Callback(hObject, eventdata, handles)
jog(-0.01, 1, handles);

% --- Executes on button press in plusY_pushbutton.
function plusY_pushbutton_Callback(hObject, eventdata, handles)
jog(+0.01, 2, handles);

% --- Executes on button press in minusY_pushbutton.
function minusY_pushbutton_Callback(hObject, eventdata, handles)
jog(-0.01, 2, handles);

% --- Executes on button press in plusZ_pushbutton.
function plusZ_pushbutton_Callback(hObject, eventdata, handles)
jog(+0.01, 3, handles);

% --- Executes on button press in minusZ_pushbutton.
function minusZ_pushbutton_Callback(hObject, eventdata, handles)
jog(-0.01, 3, handles);

% --- Executes on button press in estop.
function estop_Callback(hObject, eventdata, handles)
global estop;
estop=get(hObject,'Value');
display('ESTOP IN ON');


% --- Executes on button press in edgeShot.
function edgeShot_Callback(hObject, eventdata, handles)
% hObject    handle to edgeShot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global balltr;
global B1_vertices;
global B1;
global B2;
global B2_vertices;
global tr;
steps=20;
global estop;

if estop==0

    newTr=balltr*transl(0.5,0,-0.07);
    trAngle=troty(deg2rad(-90))*trotz(deg2rad(180));
    newTr(1:3,1:3)=trAngle(1:3,1:3);
    
    qMatrix=jtraj(handles.model.getpos(),handles.model.ikcon(newTr, handles.model.getpos()),steps);
    for i=1:steps
        handles.model.animate(qMatrix(i,:));
        tr=handles.model.fkine(handles.model.getpos())*transl(0,0,0.02);
        transformedVertices=[B1_vertices,ones(size(B1_vertices,1),1)]*tr';
        set(B1,'Vertices',transformedVertices(:,1:3));
    end
    
    pause(1);
    [qMatrix,steps]=shot(handles, 1, 8);
    
    % Plot the results
    for i=1:steps
        handles.model.animate(qMatrix(i,:));
        tr=handles.model.fkine(handles.model.getpos())*transl(0,0,0.02);
        transformedVertices=[B1_vertices,ones(size(B1_vertices,1),1)]*tr';
        set(B1,'Vertices',transformedVertices(:,1:3));
    end
    
    % ball is at edge of table 
    balltr=transl(-1.4,0,0.81);
    transformedVertices=[B2_vertices,ones(size(B2_vertices,1),1)]*balltr';
    set(B2,'Vertices',transformedVertices(:,1:3));
    
    updateJoint(handles);

else
    display('ESTOP IN ON');
end


function cornerShot_Callback(hObject, eventdata, handles)

global balltr;
global B1_vertices;
global B1;
global B2;
global B2_vertices;
global tr;
steps=20;
global estop;

if estop==0

    newTr=balltr*transl(0.5,0,-0.07);
    trAngle=troty(deg2rad(-90))*trotz(deg2rad(180));
    newTr(1:3,1:3)=trAngle(1:3,1:3)*rpy2r(deg2rad(30),deg2rad(0),deg2rad(0));
    
    qMatrix=jtraj(handles.model.getpos(),handles.model.ikcon(newTr, handles.model.getpos()),steps);
    for i=1:steps
        handles.model.animate(qMatrix(i,:));
        tr=handles.model.fkine(handles.model.getpos())*transl(0,0,0.02);
        transformedVertices=[B1_vertices,ones(size(B1_vertices,1),1)]*tr';
        set(B1,'Vertices',transformedVertices(:,1:3));
    end
    
    pause(1);
    [qMatrix,steps]=shot(handles, 0, 8);
    
    % Plot the results
    for i=1:steps
        handles.model.animate(qMatrix(i,:));
        tr=handles.model.fkine(handles.model.getpos())*transl(0,0,0.02);
        transformedVertices=[B1_vertices,ones(size(B1_vertices,1),1)]*tr';
        set(B1,'Vertices',transformedVertices(:,1:3));
    end
    % ball is at edge of table 
    balltr=transl(-1.37,0.75,0.81);
    transformedVertices=[B2_vertices,ones(size(B2_vertices,1),1)]*balltr';
    set(B2,'Vertices',transformedVertices(:,1:3));

    updateJoint(handles);

else
    display('ESTOP IN ON');
end

% --- Executes on button press in cornerShot2.
function cornerShot2_Callback(hObject, eventdata, handles)
global balltr;
global B1_vertices;
global B1;
global B2;
global B2_vertices;
global tr;
steps=20;
global estop;

if estop==0

    newTr=balltr*transl(0.5,0,-0.07);
    trAngle=troty(deg2rad(-90))*trotz(deg2rad(180));
    newTr(1:3,1:3)=trAngle(1:3,1:3)*rpy2r(deg2rad(-30),deg2rad(0),deg2rad(0));
    
    qMatrix=jtraj(handles.model.getpos(),handles.model.ikcon(newTr, handles.model.getpos()),steps);
    for i=1:steps
        handles.model.animate(qMatrix(i,:));
        tr=handles.model.fkine(handles.model.getpos())*transl(0,0,0.02);
        transformedVertices=[B1_vertices,ones(size(B1_vertices,1),1)]*tr';
        set(B1,'Vertices',transformedVertices(:,1:3));
    end
    
    pause(1);
    [qMatrix,steps]=shot(handles, 0, 8);
    
    % Plot the results
    for i=1:steps
        handles.model.animate(qMatrix(i,:));
        tr=handles.model.fkine(handles.model.getpos())*transl(0,0,0.02);
        transformedVertices=[B1_vertices,ones(size(B1_vertices,1),1)]*tr';
        set(B1,'Vertices',transformedVertices(:,1:3));
    end
    % ball is at edge of table 
    balltr=transl(-1.37,-0.75,0.81);
    transformedVertices=[B2_vertices,ones(size(B2_vertices,1),1)]*balltr';
    set(B2,'Vertices',transformedVertices(:,1:3));

    updateJoint(handles);

else
    display('ESTOP IN ON');
end

% --- Executes on button press in allign.
function allign_Callback(hObject, eventdata, handles)
% hObject    handle to allign (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global B1_vertices;
global B1;
global estop;

if estop==0

    % Create image target (points in the image plane) 
    pStar = [662 362 362 662; 362 362 662 662];
    %Create 3D points
    P=[1.3,1.3,1.2,1.2;
    -0.6 ,-0.5,-0.5,-0.6;
    0.8,0.8,0.8,0.8];
    q0 = [deg2rad(50); deg2rad(20); deg2rad(40); deg2rad(-70); deg2rad(-80); deg2rad(-45)];
    
    % Add the camera
    cam = CentralCamera('focal', 0.08, 'pixel', 10e-5,'resolution', [1024 1024], 'centre', [512 512],'name', 'TM5camera');
    fps = 25;% frame rate
    %Define values
    lambda = 0.7; %gain of the controler
    depth = mean (P(3,:)); %depth of the IBVS
    % 1.2 Initialise Simulation (Display in 3D)
    Tc0= handles.model.fkine(q0);
    handles.model.animate(q0');
    drawnow
    % Display points in 3D and the camera
    cam.plot_camera('Tcam',Tc0, 'label','scale',0.15);
    plot_sphere(P, 0.02, 'r')
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
    maxSteps=130;
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
            J2 = handles.model.jacobn(q0);
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
    
            %for i=1:ksteps
            handles.model.animate(q');
            tr=handles.model.fkine(handles.model.getpos())*transl(0,0,0.02);
            transformedVertices=[B1_vertices,ones(size(B1_vertices,1),1)]*tr';
            set(B1,'Vertices',transformedVertices(:,1:3));
            %end
    
            %Get camera location
            Tc = handles.model.fkine(q);
            cam.T = Tc;
            drawnow
            if ~isempty(maxSteps) && (ksteps > maxSteps)
                break;
            end
            %update current joint position
            q0 = q;
     end
 
    updateJoint(handles);

else
    display('ESTOP IN ON');
end

% --- Executes on slider movement.
function joint1Slider_Callback(hObject, eventdata, handles)

global estop;
if estop==0
    j1=get(hObject,'Value');
    set(handles.j1Text,'String',j1);
    q=handles.model.getpos();
    q(1)=deg2rad(j1);
    tr = handles.model.fkine(q);
    handles.model.animate(q);
else
    display('ESTOP IN ON');
end
updateJoint(handles);

% --- Executes during object creation, after setting all properties.
function joint1Slider_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function joint2Slider_Callback(hObject, eventdata, handles)

global estop;
if estop==0
    j2=get(hObject,'Value');
    set(handles.j2Text,'String',j2);
    q=handles.model.getpos();
    q(2)=deg2rad(j2);
    tr = handles.model.fkine(q);
    handles.model.animate(q);
else
    display('ESTOP IN ON');
end
updateJoint(handles);

% --- Executes during object creation, after setting all properties.
function textjoint2Slidetextr_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function joint3Slider_Callback(hObject, eventdata, handles)

global estop;
if estop==0
    j3=get(hObject,'Value');
    set(handles.j3Text,'String',j3);
    q=handles.model.getpos();
    q(3)=deg2rad(j3);
    tr = handles.model.fkine(q);
    handles.model.animate(q);
else
    display('ESTOP IN ON');
end
updateJoint(handles);

% --- Executes during object creation, after setting all properties.
function joint3Slider_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function joint4Slider_Callback(hObject, eventdata, handles)

global estop;
if estop==0
    j4=get(hObject,'Value');
    set(handles.j4Text,'String',j4);
    q=handles.model.getpos();
    q(4)=deg2rad(j4);
    tr = handles.model.fkine(q);
    handles.model.animate(q);
else
    display('ESTOP IN ON');
end
updateJoint(handles);

% --- Executes during object creation, after setting all properties.
function joint4Slider_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function joint5Slider_Callback(hObject, eventdata, handles)

global estop;
if estop==0
    j5=get(hObject,'Value');
    set(handles.j5Text,'String',j5);
    q=handles.model.getpos();
    q(5)=deg2rad(j5);
    tr = handles.model.fkine(q);
    handles.model.animate(q);
else
    display('ESTOP IN ON');
end
updateJoint(handles);

% --- Executes during object creation, after setting all properties.
function joint5Slider_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function joint6Slider_Callback(hObject, eventdata, handles)

global estop;
if estop==0
    j6=get(hObject,'Value');
    set(handles.j6Text,'String',j6);
    q=handles.model.getpos();
    q(6)=deg2rad(j6);
    tr = handles.model.fkine(q);
    handles.model.animate(q);
else
    display('ESTOP IN ON');
end
updateJoint(handles);
%jointJog(6, handles, hObject);

% --- Executes during object creation, after setting all properties.
function joint6Slider_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in reset.
function reset_Callback(hObject, eventdata, handles)

global balltr;
global B1_vertices;
global B1;
global B2;
global B2_vertices;
global tr;

%reset robot joint angles
q=[0 0 0 0 0 0];
handles.model.animate(q);

% reset ping pong racket
tr=handles.model.fkine(handles.model.getpos())*transl(0,0,0.02);
transformedVertices=[B1_vertices,ones(size(B1_vertices,1),1)]*tr';
set(B1,'Vertices',transformedVertices(:,1:3));

% reset ball location
balltr=transl(0.8,0,1.4);
transformedVertices=[B2_vertices,ones(size(B2_vertices,1),1)]*balltr';
set(B2,'Vertices',transformedVertices(:,1:3));

%if box is present, delete
% try
%     delete('Box.ply')
% end



% --- Executes on button press in box.
function box_Callback(hObject, eventdata, handles)
PlaceObject('box.ply', [0.8 0 0.8])




% --- Executes on button press in collisionDetectionEdgeShot.
function collisionDetectionEdgeShot_Callback(hObject, eventdata, handles)
% collion detection to show when moving the robot forward
global balltr;
steps=20;
global B1_vertices;
global B1;
global collisionFlag;
collisionFlag=0;

% Start with robot holding paddle as if it's about to perform edge shot
newTr=balltr*transl(0.5,0,-0.07);
trAngle=troty(deg2rad(-90))*trotz(deg2rad(180));
newTr(1:3,1:3)=trAngle(1:3,1:3);
qMatrix=jtraj(handles.model.getpos(),handles.model.ikcon(newTr, handles.model.getpos()),steps);

for i = 1: steps
    handles.model.animate(qMatrix(i,:));
    % move ping pong paddle
    tr=handles.model.fkine(handles.model.getpos())*transl(0,0,0.02);
    transformedVertices=[B1_vertices,ones(size(B1_vertices,1),1)]*tr';
    set(B1,'Vertices',transformedVertices(:,1:3));
end

% % Find normals, vertices and faces of that prism. Each faceNormal represents a triangle
[faces,vertex] = plyread('box.ply','tri')
%PlaceObject('box.ply', [0.9 0 0.8]) therefore translate vertices:
vertex
vertex(:,1)=vertex(:,1)+0.8
vertex(:,3)=vertex(:,3)+0.8
vertex
for i=1:12
    n1=faces(i,1);
    v1=vertex(n1,:);
    n2=faces(i,2);
    v2=vertex(n2,:);
    n3=faces(i,3);
    v3=vertex(n3,:);

    faceNormals(i,:)=unit(cross(v2-v1,v3-v1))
end

pause(3);

% 2.6: Go through 'forward' movement of robot with ping pong paddle:
newTr=balltr*transl(0.5,0,-0.07);
trAngle=troty(deg2rad(-90))*trotz(deg2rad(180));
newTr(1:3,1:3)=trAngle(1:3,1:3);
NEWqMatrix=jtraj(handles.model.getpos(),handles.model.ikcon(newTr*transl(0,0,0.6), handles.model.getpos()),steps);
display("Hello")

% 2.7
result = true(steps,1);
for i = 1: steps
    disp(i);
    result(i) = IsCollision(handles.model,NEWqMatrix(i,:),faces,vertex,faceNormals,false);

    if collisionFlag == 1
        display("Collision detected in next step, exiting");
        break;
    end

    handles.model.animate(NEWqMatrix(i,:));
    % move ping pong paddle
    tr=handles.model.fkine(handles.model.getpos())*transl(0,0,0.02);
    transformedVertices=[B1_vertices,ones(size(B1_vertices,1),1)]*tr';
    set(B1,'Vertices',transformedVertices(:,1:3));
end