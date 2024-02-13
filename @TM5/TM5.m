classdef TM5 < handle
    properties
        
        %> Robot model
        model;
        %>
        workspace = [-2.5 2.5 -2.5 2.5 0 2.5];   
        %> Flag to indicate if gripper is used
        useGripper = false;        
    end
    
    methods%% Class for TM5 robot simulation
function self = TM5(useGripper)
    if nargin < 1
        useGripper = false;
    end
    self.useGripper = useGripper;
    
%> Define the boundaries of the workspace
        
% robot = 
self.GetTM5Robot();
% robot = 
 self.PlotAndColourRobot();%robot,workspace);
end
%% GetTM5Robot
% Given a name (optional), create and return a TM5 robot model
function GetTM5Robot(self)
%     if nargin < 1
        % Create a unique name (ms timestamp after 1ms pause)
        pause(0.001);
        name = ['TM5',datestr(now,'yyyymmddTHHMMSSFFF')];
%     end

L1 = Link('d',0.1452,'a',0,'alpha',pi/2,'qlim',[-deg2rad(270) deg2rad(270)]);
L2 = Link('d',0,'a',-0.329,'alpha',0,'qlim',[-pi pi], 'offset', -pi/2);
L3 = Link('d',0,'a',-0.311,'alpha',0,'qlim',[-deg2rad(155) deg2rad(155)]);
L4 = Link('d',0.106,'a',0,'alpha',pi/2,'qlim',[-pi pi]);
L5 = Link('d',0.106,'a',0,'alpha',-pi/2,'qlim',[-pi pi]);
L6 = Link('d',0.1132,'a',0,'alpha',0,'qlim',[-deg2rad(270) deg2rad(270)]);

self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);
end
%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self)%robot,workspace)
    for linkIndex = 0:self.model.n
        if self.useGripper && linkIndex == self.model.n
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['TM5Link',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
        else
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['TM5Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        end
        self.model.faces{linkIndex+1} = faceData;
        self.model.points{linkIndex+1} = vertexData;
    end

    % Display robot
    self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end  
    self.model.delay = 0;

    % Try to correctly colour the arm (if colours are in ply file data)
    for linkIndex = 0:self.model.n
        handles = findobj('Tag', self.model.name);
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
end        
    end
end