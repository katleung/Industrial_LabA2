classdef gripper < RobotBaseClassEdited
    %% LinearUR3 gripper FINGER
    properties(Access = public)   
        plyFileNameStem = 'gripper';
    end
    
    methods
%% Constructor
        function self = gripper(baseTr)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);  
                end             
            else % All passed in 
                % self.useTool = useTool;
                % toolTrData = load([toolFilename,'.mat']);
                % self.toolTr = toolTrData.tool;
                % self.toolFilename = [toolFilename,'.ply'];
            end
            
            self.CreateModel();
			self.model.base = self.model.base.T * baseTr
            %self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
            drawnow
        end

%% CreateModel
        function CreateModel(self)
            % Set DH Parameters for the gripper model
            link(1) = Link('d',0,'a',0.06,'alpha',0,'qlim',deg2rad([-50 0]),'offset',deg2rad(160));
            link(2) = Link('d',0,'a',-0.04,'alpha',0,'qlim',deg2rad([-40 20]),'offset',deg2rad(160));
            link(3) = Link('d',0,'a',-0.002,'alpha',0,'qlim',deg2rad([-30 30]),'offset',deg2rad(60));
            self.model = SerialLink(link,'name',self.name);
        end 

    end
end