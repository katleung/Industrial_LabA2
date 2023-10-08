classdef teaUR3 < RobotBaseClassEdited
    %% UR3 Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'teaUR3';
    end
    
    methods
%% Constructor
        function self = teaUR3(baseTr) 
            close all
            % note: no tools passed in. gripper is separate because it is two 3-link serial robot
            if nargin < 1
    			baseTr = eye(4); % If there are no input arguments, the base is assumed to be at the origin
            end
            
            self.CreateModel();
			self.model.base = self.model.base.T * baseTr
            %self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
            drawnow
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',-pi/2);
            link(3) = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            link(4) = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', -pi/2);
            link(5) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(6) = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
            
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
