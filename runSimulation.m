%% THIS CODE WAS DEVELOPED BY BRIANNA HADIWIJAYA (13560002), KATRINA LEUNG (13570181), EMMA QUIGLEY (13609641)
classdef runSimulation < handle
    properties
        baseTr = eye(4);
        cupbot;                                 % Create a robot object
        gripper = {};                           % Create a gripper object
        jtrajStepCount = 100;                   % Number of trajectory points for animation

        cupStartLoc = zeros(1,3);               % Initialise all cup startLocation with zeros
        cupStatus = zeros(1,1);                 % Initialise all cup statuses to 0 (empty)
        cupEndLoc = zeros(1,3);                 % Initialise all cup endLocations with zeros
        cupID = {};                             % Initialise empty cell array to store cup handles (so that they can be deleted later)
        cupFillLoc = [0.231, 0.333, 0.453];     % Set filling location for cup
        cupResetLoc = [0.231, 0.333, 0.453];    % Set reset location for cupbot to avoid hitting the next cup
        armToCupOffset = 0.05; 

        rotateEnd = [1 0 0; 0 -1 0; 0 0 -1]; % rotation matrix to make EE face downwards

        closedGrip = deg2rad([-27.6, -14.1,-5.88]);         % Set pose for closed gripper position
        openGrip = deg2rad([-10 -10 0]);                    % Set pose for open gripper position

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
            self.gripper{1} = gripper(self.cupbot.model.fkine(self.cupbot.model.getpos()).T * trotx(pi/2));
            self.gripper{2} = gripper(self.cupbot.model.fkine(self.cupbot.model.getpos()).T * trotx(pi/2) * troty(pi));
            self.gripper{1}.model.animate(self.openGrip); 
            self.gripper{2}.model.animate(self.openGrip);
            self.moveFingers(self, 1, 20);

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
            % cupID = cell(1, 1);
            cupStartLoc(1,:) = [-0.023, 0.569, 0.253];
            % cupStartLoc(1,:) = [0.160, -0.505, 0.296];
            % cupStartLoc(3,:) = [-0.023, 0.569, 0.253];
            
            for n = 1:height(cupStartLoc) % for number of rows in cup array
                updateLoc = transl(cupStartLoc(n,:)) * self.baseTr;
                cupStartLoc(n,:) = [updateLoc(1,4), updateLoc(2,4), updateLoc(3,4)];
                cupID{n} = PlaceObject('HalfSizedRedGreenBrick.ply', cupStartLoc(n,:));
            end 
        end

        %% Set end locations for cups
        function cupEndLoc = cupEndLocationSet(self)
            cupEndLoc = zeros(1,3);
            endLoc(1,:) = self.cupStartLoc(1,:); % return cup to its original position 
            % endLoc(2,:) = [0.231, 0.333, 0.453];
            % endLoc(3,:) = [0.231, 0.333, 0.453];

            for n = 1:height(endLoc) % for number of rows in cup array
                updateLoc = transl(endLoc(n,:)) * self.baseTr;
                cupEndLoc(n,:) = [updateLoc(1,4), updateLoc(2,4), updateLoc(3,4)];
            end 
        end
        
        %% Move Cup (build wall)
        function buildWall(self)
            % Set the initial joint angles to the current robot position
            q = self.cupbot.model.getpos()
            q= deg2rad([20,1,20,1,0,0])
        
            for i = 1:height(self.cupID)
                while self.cupStatus(i) <4
                    if self.cupStatus(i) == 0 %move to cup start location, close gripper
                        goalMatrix = rt2tr(self.rotateEnd, self.cupStartLoc(i,:)');
                        goalQ = self.cupbot.model.ikine(goalMatrix, 'q0', q, 'mask', [1,1,1,0,0,0]);
                        goalPos = self.cupbot.model.fkine(goalQ).T;
                        self.moveCupbotNOCup(self,goalQ,self.jtrajStepCount);
                        self.moveFingers(self, 0, 20);
                    elseif self.cupStatus(i) == 1 %move cup to fill location, keep gripper closed
                        goalMatrix = rt2tr(self.rotateEnd, self.cupFillLoc(1,:)');       % there should only be one fill loc. 
                        goalQ = self.cupbot.model.ikine(goalMatrix, 'q0', q, 'mask', [1,1,1,0,0,0]);
                        goalPos = self.cupbot.model.fkine(goalQ).T;
                        self.moveCupbotWITHCup(self,i,goalQ,self.jtrajStepCount);
                    elseif self.cupStatus(i) == 2 %move cup to end location, open gripper
                        goalMatrix = rt2tr(self.rotateEnd, self.cupEndLoc(i,:)');
                        goalQ = self.cupbot.model.ikine(goalMatrix, 'q0', q, 'mask', [1,1,1,0,0,0]);
                        goalPos = self.cupbot.model.fkine(goalQ).T;
                        self.moveCupbotWITHCup(self,i,goalQ,self.jtrajStepCount);
                        self.moveFingers(self, 1, 20);
                    elseif self.cupStatus(i) == 3 %move arm without cup to reset position, keep gripper open
                        goalMatrix = rt2tr(self.rotateEnd, self.cupResetLoc(1,:)');
                        goalQ = self.cupbot.model.ikine(goalMatrix, 'q0', q, 'mask', [1,1,1,0,0,0]);
                        goalPos = self.cupbot.model.fkine(goalQ).T;
                        self.moveCupbotNOCup(self,goalQ,self.jtrajStepCount);
                    end

                    self.cupStatus(i) = self.cupStatus(i) + 1; 

                end
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
                self.transformGripper(self); % Animate the gripper transform by one step to the same end-effector position
                drawnow();
                pause(0.005);
            end
        end

        %% Move UR3 with cup
        function moveCupbotWITHCup(self, iD, goal, stepCount)
            currentPose = self.cupbot.model.getpos(); % initialise a variable with the right number of joint values
            steps = jtraj(self.cupbot.model.getpos(), goal, stepCount);
            
            for j = 1:stepCount
                currentPose = self.cupbot.model.getpos(); % initialise a variable with the right number of joint values
                trEnd = self.cupbot.model.fkine(currentPose).T;
                currentPose(1:3) = steps(j, 1:3); % Only update the position part of the pose
                self.cupbot.model.animate(currentPose) % Animate the robot's movement
                self.transformGripper(self); % Animate the gripper transform by one step to the same end-effector position

                try 
                    delete(self.cupID{iD}); 
                catch 
                end

                cupPos = trEnd * transl(0,0,self.armToCupOffset);
                self.cupID{iD} = PlaceObject('HalfSizedRedGreenBrick.ply', cupPos(1:3, 4)');

                drawnow();
                pause(0.005);
            end

        end

        %% Move gripper fingers
        function moveFingers(self, state, stepCount)
            if state == 1 % close to open
                fingerTraj = jtraj(self.closedGrip, self.openGrip, stepCount); % Calculate trajectory of opening the grip
            elseif state == 0 % open to close
                fingerTraj = jtraj(self.openGrip, self.closedGrip, stepCount); % Calcualte the trajectory of closigng the grip
            end

            % Animate each finger following the open/close trajectory 
            for j = 1:size(fingerTraj, 1) 
                self.gripper{1}.model.animate(fingerTraj(j, :)); 
                self.gripper{2}.model.animate(fingerTraj(j, :)); 
                drawnow();
            end
        end

        %% Transform whole gripper
        function transformGripper(self)
            startTr1 = self.gripper{1}.model.fkine(self.gripper{1}.model.getpos()).T; % current position of gripper base
            endTr1 = self.cupbot.model.fkine(self.cupbot.model.getpos()).T * trotx(pi/2); % current position of robot end-effector (i.e. goal position for gripper 1)
            trPath1 = ctraj(startTr1, endTr1, 2); % Use ctraj to define a trajectory that accounts for translation as well from startTr1 to endTr1

            % Calculate the new endTr2 based on endTr1 rotated by troty(pi)
            endTr2 = endTr1 * troty(pi); % goal position of gripper 2 is the same as gripper 1 but rotated about y to mirror the finger
            trPath2 = ctraj(startTr1, endTr2, 2);  % Use ctraj to define a trajectory that accounts for translation as well from startTr1 to endTr2

            for i = 1:size(trPath1, 3)
                % % Animate gripper{1} from startTr1 to endTr1
                self.gripper{1}.model.base = trPath1(:, :, i);
                self.gripper{1}.model.animate(self.gripper{1}.model.getpos);
                % Animate gripper{2} from startTr1 to endTr2
                self.gripper{2}.model.base = trPath2(:, :, i);
                self.gripper{2}.model.animate(self.gripper{2}.model.getpos);
            end
        end

    end
end
