%% THIS CODE WAS DEVELOPED BY KATRINA LEUNG (13570181)
classdef runSimulation < handle
    properties
        baseTr = eye(4);
        cupbot;                             % Create a robot object
        jtrajStepCount = 100;                % Number of trajectory points for animation

        cupStartLoc = zeros(1, 3);                  % Initialise all cup startLocation with zeros
        cupStatus = zeros(1,1);           % Initialise all cup statuses to 0 (empty)
        cupEndLoc = zeros(2, 3);          % Initialise all cup endLocations with zeros
        cupID = {};                         % Initialise empty cell array to store cup handles (so that they can be deleted later)

        rotateEnd = [1 0 0; 0 -1 0; 0 0 -1]; % rotation matrix to make EE face downwards

    end

    methods (Static)
        %% Constructor - Load Environment
        function self = runSimulation(baseTransform)
            close all
            if nargin > 0  % Check if a base transformation matrix is provided
                self.baseTr = self.baseTr * baseTransform; %apply transformation if provided
            end

            %load in environment
            self.LoadEnviro(self);

            %load in the robot
            self.cupbot = teaUR3(self.baseTr);
            
            %load bricks/cups
            [self.cupID, self.cupStartLoc] = self.loadCups(self); % For testing within the class, without starterScript
            self.cupEndLoc = self.cupEndLocationSet(self)

            %buildwall
            self.buildWall(self);

        end
        
        %% Loading the Environment
         function LoadEnviro(self)
            % load in surface texture for concrete floor
            surf([-1.5,-1.5;1.5,1.5],[-1.5,1.5;-1.5,1.5],[0,0;0,0],'CData',imread('concrete.jpg'),'FaceColor','texturemap');    %Load concrete floor
            hold on;
         
         end

        %% Generating Cup / Cups?
        function [cupID, cupStartLoc] = loadCups(self)
            cupStartLoc = zeros(1,3);
            cupID = cell(1, 1);
            cupStartLoc(1,:) = [-0.023, 0.569, 0.253];
            for n = 1:height(cupStartLoc) % for number of rows in cup array
                updateLoc = transl(cupStartLoc(n,:)) * self.baseTr;
                cupStartLoc(n,:) = [updateLoc(1,4), updateLoc(2,4), updateLoc(3,4)];
                % cupID{n} = PlaceObject('HalfSizedRedGreenBrick.ply', cupStartLoc(n,:));
            end 
        end

        %% Set end locations for cups
        function cupEndLoc = cupEndLocationSet(self)
            cupEndLoc = zeros(1,3);
            endLoc(1,:) = self.cupStartLoc(1,:);
            endLoc(2,:) = [0.231, 0.333, 0.453];

            for n = 1:height(endLoc) % for number of rows in cup array
                updateLoc = transl(endLoc(n,:)) * self.baseTr;
                cupEndLoc(n,:) = [updateLoc(1,4), updateLoc(2,4), updateLoc(3,4)];
            end 
        end
        %% Move Cup (build wall)
        function buildWall(self)
            % Set the initial joint angles to the current robot position
            q = self.cupbot.model.getpos()

            for i = 1: height(self.cupEndLoc)
                goalMatrix = rt2tr(self.rotateEnd, self.cupEndLoc(i,:)')
                goalQ = self.cupbot.model.ikine(goalMatrix, 'q0', q, 'mask', [1,1,1,0,0,0])
                goalPos = self.cupbot.model.fkine(goalQ).T;
                self.moveCupbotNOCup(self,goalQ,self.jtrajStepCount);
            end
        end

        %% Move UR3 without cup
        function moveCupbotNOCup(self, goal, stepCount)
            % goalPosition = self.robot.model.fkine(goalQ).T;
            currentPose = self.cupbot.model.getpos();
            steps = jtraj(self.cupbot.model.getpos(), goal, stepCount);
            
            for j = 1:stepCount
                currentPose(1:3) = steps(j, 1:3); % Only update the position part of the pose
                self.cupbot.model.animate(currentPose) % Animate the robot's movement
                drawnow();
                pause(0.005);
            end
        end

    end
end
