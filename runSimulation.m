%% THIS CODE WAS DEVELOPED BY BRIANNA HADIWIJAYA (13560002), KATRINA LEUNG (13570181), EMMA QUIGLEY (13609641)
classdef runSimulation < handle
    properties
        baseTr = eye(4);
        cupbot;                                 % Create a robot object
        irb;
        gripper = {};                           % Create a gripper object
        jtrajStepCount = 100;                   % Number of trajectory points for animation
        
        irbOffset = transl([0.95,-0.323,0.25])*trotz(-pi/2);
        toppingsLocation = [0.95,-1.022,0.113];
        cupStartLoc = zeros(1,3);               % Initialise all cup startLocation with zeros
        cupStatus = zeros(1,1);                 % Initialise all cup statuses to 0 (empty)
        cupEndLoc = zeros(1,3);                 % Initialise all cup endLocations with zeros
        cupID = {};                             % Initialise empty cell array to store cup handles (so that they can be deleted later)
        cupFillLoc = [0.4, -0.18, 0.518];     % Set filling location for cup
        irbCupFillLoc = [0.35, -0.19, 0.647];
        cupResetLoc = [0.231, 0.333, 0.453];    % Set reset location for cupbot to avoid hitting the next cup
        armToCupOffset = 0.05; 
        
        rotateEnd = [1 0 0; 0 0 -1; 0 1 0]; % rotation matrix to make EE face downwards

        closedGrip = deg2rad([-27.6, -14.1,-5.88]);         % Set pose for closed gripper position
        openGrip = deg2rad([-10 -10 0]);                    % Set pose for open gripper position

        Arduino;
    end

    methods (Static)
        %% Constructor - Load Environment
        function self = runSimulation(baseTransform)
            close all
            if nargin > 0  % Check if a base transformation matrix is provided
                self.baseTr = self.baseTr * baseTransform; %apply transformation if provided
            end
            
            %load in the robot

            self.cupbot = teaUR3(self.baseTr);
            self.irb = IRB1200(self.irbOffset);
            self.gripper{1} = gripper(self.cupbot.model.fkine(self.cupbot.model.getpos()).T * trotx(pi/2));
            self.gripper{2} = gripper(self.cupbot.model.fkine(self.cupbot.model.getpos()).T * trotx(pi/2) * troty(pi));
            self.gripper{1}.model.animate(self.openGrip); 
            self.gripper{2}.model.animate(self.openGrip);
            self.moveFingers(self, 1, 20);
            hold on;
            
            % Set robot starting position
            q = self.irb.model.getpos;
            endJoint = deg2rad([0 -38 5 0 -40 0]);
            steps = 20;
            trajectory = jtraj(q,endJoint,steps);
            for i = 1:steps
                self.irb.model.animate(trajectory(i,:)); % Animating the robot to move to the set joint configuratio
                drawnow()
                pause(0.005);
            end

            %load bricks/cups
            [self.cupID, self.cupStartLoc] = self.loadCups(self); % For testing within the class, without starterScript
            self.cupEndLoc = self.cupEndLocationSet(self);
            
            %load in environment
            self.LoadEnviro(self);

            %buildwall
            self.buildWall(self);

            
        end
        
        %% Loading the Environment
         function LoadEnviro(self)
            % load in surface texture for concrete floor
            surf([-4,-4;4,4],[-4,4;-4,4],[-0.152,-0.152;-0.152,-0.152],'CData',imread('Models\marble.jpg'),'FaceColor','texturemap');    %Load concrete floor
            hold on;
            PlaceObject('Models\bowl.ply', [0.95,-0.9,0]);
            hold on;
            SafetySystem.ActivateAll; 
            PlaceObject('Models\CoffeeTable2.ply',[0.6, 0.3, -0.152]);
            PlaceObject('Models\CoffeeTable3.ply',[0.95,-0.9, -0.152]);
            PlaceObject('Models\mount3.ply',[0, 0, 0]);
            PlaceObject('Models\mount4.ply',[0.95,-0.323,0.1913-0.152]);
            


         
         end

        %% Generating Cup / Cups?
        function [cupID, cupStartLoc] = loadCups(self)
            cupStartLoc = zeros(1,3);

            cupStartLoc(1,:) = [0.2727, 0.45, 0.2503];
            cupStartLoc(2,:) = [0.2727, 0.3, 0.2503];
            cupStartLoc(3,:) = [0.2727, 0.15, 0.2503];

            
            for n = 1:height(cupStartLoc) % for number of rows in cup array
                updateLoc = transl(cupStartLoc(n,:)) * self.baseTr;
                cupStartLoc(n,:) = [updateLoc(1,4), updateLoc(2,4), updateLoc(3,4)];
                cupID{n} = PlaceObject('Models\cup2.ply', cupStartLoc(n,:));
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
            q = self.cupbot.model.getpos();
            offsetPose = transl(-self.armToCupOffset, 0, 0)* trotx(pi);
            offsetWaypoint = transl(-self.armToCupOffset, 0, 0.2)* trotx(pi);
            [m,n] = size(self.cupID)
            for i = 1:n
                self.cupStatus(i) = 0;
                while self.cupStatus(i) <4
                    if self.cupStatus(i) == -1
                        goalQ = deg2rad([-90,-45,45,-90,90,0]);
                        goalPos = self.cupbot.model.fkine(goalQ).T;
                        self.moveCupbotNOCup(self,goalQ,self.jtrajStepCount);
                    elseif self.cupStatus(i) == 0 % move to cup start location, close gripper
                        totalSteps = 40;
                        ur3FirstSteps = 30;
                        ur3SecondSteps = totalSteps-ur3FirstSteps;
                        self.cupbot.model.getpos
                        % startCupLocationWaypoint = transl(self.cupStartLoc(i,:))*offsetWaypoint;
                        
                        % trajectory to move to waypoint above cup
                        goalQAboveCup = self.cupbot.model.ikine(transl(self.cupStartLoc(i,:))*offsetWaypoint, 'q0', self.cupbot.model.getpos, 'mask', [1,1,1,0,0,0]);
                        trajectoryAboveCup = jtraj(self.cupbot.model.getpos, goalQAboveCup,ur3FirstSteps);
                        % trajectory to move from waypoint down to cup
                        startCupLocation = transl(self.cupStartLoc(i,:))*offsetPose;
                        trajectoryToCup = self.rmrc(self, goalQAboveCup, [startCupLocation(1,4), startCupLocation(2,4), startCupLocation(3,4)], ur3SecondSteps, self.cupbot);
                        % total UR3 trajectory
                        ur3trajectoryToEmptyCup = [trajectoryAboveCup;trajectoryToCup];
                        % irb trajectory
                        trajectoryToToppings = self.rmrc(self, self.irb.model.getpos, self.toppingsLocation, totalSteps, self.irb);
                        % animation
                        self.animateBothRobotsNoCup(self,ur3trajectoryToEmptyCup, trajectoryToToppings, totalSteps);
                        % close gripper fingers
                        self.moveFingers(self, 0, 20);
                        self.pickupToppings(self);
                    elseif self.cupStatus(i) == 1 % move cup to fill location, keep gripper closed
                        totalSteps = 40;
                        ur3FirstSteps = 30;
                        ur3SecondSteps = totalSteps-ur3FirstSteps;
                        % move to waypoint above cup
                        % goalQ = self.cupbot.model.ikine(transl(self.cupStartLoc(i,:))*offsetWaypoint, 'q0', q, 'mask', [1,1,1,0,0,0])
                        % self.moveCupbotWITHCup(self,i,goalQ,self.jtrajStepCount);
                        startCupLocationWaypoint = transl(self.cupStartLoc(i,:))*offsetWaypoint;
                        trajectoryAboveCup = self.rmrc(self, self.cupbot.model.getpos, [startCupLocationWaypoint(1,4), startCupLocationWaypoint(2,4), startCupLocationWaypoint(3,4)], ur3SecondSteps, self.cupbot);
                        % move to fill location
                        % goalQ = self.cupbot.model.ikine(transl(self.cupFillLoc(i,:))*offsetPose, 'q0', trajectoryAboveCup(end,:), 'mask', [1,1,1,0,0,0]);
                        % trajectoryToToppings = jtraj(trajectoryAboveCup(end,:), goalQ,ur3FirstSteps);
                        fillLocationPoint = transl(self.cupFillLoc(1,:))*offsetPose;
                        trajectoryToToppings = self.rmrc(self, trajectoryAboveCup(end,:), [fillLocationPoint(1,4), fillLocationPoint(2,4), fillLocationPoint(3,4)], ur3FirstSteps, self.cupbot);
                        % total UR3 trajectory
                        ur3trajectoryToToppings = [trajectoryAboveCup;trajectoryToToppings];
                        % irb trajectory
                        trajectoryToCup = self.rmrc(self, self.irb.model.getpos, self.irbCupFillLoc, totalSteps, self.irb);
                        self.animateBothRobotsEmptyCup(self,ur3trajectoryToToppings,trajectoryToCup,totalSteps,i);
                        self.dispenseToppings(self);
                    elseif self.cupStatus(i) == 2 %move cup to end location, open gripper
                        totalSteps = 40;
                        ur3FirstSteps = 20;
                        ur3SecondSteps = totalSteps-ur3FirstSteps;
                        % move to waypoint above cup
                        startCupLocationWaypoint = transl(self.cupStartLoc(i,:))*offsetWaypoint;
                        trajectoryAboveCup = self.rmrc(self, self.cupbot.model.getpos, [startCupLocationWaypoint(1,4), startCupLocationWaypoint(2,4), startCupLocationWaypoint(3,4)], ur3SecondSteps, self.cupbot);
                        % move down to end location
                        startCupLocation = transl(self.cupStartLoc(i,:))*offsetPose;
                        trajectoryToCup = self.rmrc(self, trajectoryAboveCup(end,:), [startCupLocation(1,4), startCupLocation(2,4), startCupLocation(3,4)], ur3SecondSteps, self.cupbot);
                        % total trajectory
                        ur3trajectoryToEndLocation = [trajectoryAboveCup;trajectoryToCup];
                        self.animateUR3WithCup(self,ur3trajectoryToEndLocation,totalSteps, i);
                        self.moveFingers(self, 1, 20);
                        % move to waypoint above cup
                        steps = 10;
                        startCupLocationWaypoint = transl(self.cupStartLoc(i,:))*offsetWaypoint;
                        trajectoryAboveCup = self.rmrc(self, self.cupbot.model.getpos, [startCupLocationWaypoint(1,4), startCupLocationWaypoint(2,4), startCupLocationWaypoint(3,4)], ur3SecondSteps, self.cupbot);
                        self.animateUR3(self,trajectoryAboveCup,steps);
                    elseif self.cupStatus(i) == 3 %move arm without cup to reset position, keep gripper open
                        steps = 10;
                        resetLocation =transl(self.cupResetLoc(1,:))*offsetPose;
                        trajectoryToReset = self.rmrc(self, self.cupbot.model.getpos, [resetLocation(1,4), resetLocation(2,4), resetLocation(3,4)], steps, self.cupbot);
                        self.animateUR3(self,trajectoryToReset,steps);
                    end

                    self.cupStatus(i) = self.cupStatus(i) + 1; 

                end
            end

        end

        

        %% UR3 and IRB animation without cup
        function animateBothRobotsNoCup(self, ur3traj, irbTraj, stepCount)            
            for i = 1:stepCount
                self.irb.model.animate(irbTraj(i,:))
                self.cupbot.model.animate(ur3traj(i,:)) % Animate the robot's movement
                self.transformGripper(self); % Animate the gripper transform by one step to the same end-effector position
                drawnow();
                pause(0.01);
            end
        end

        %% UR3 and IRB animation with empty cups
        function animateBothRobotsEmptyCup(self, ur3traj, irbTraj, stepCount, iD)            
            for i = 1:stepCount
                self.irb.model.animate(irbTraj(i,:))
                self.cupbot.model.animate(ur3traj(i,:)) % Animate the robot's movement
                self.transformGripper(self); % Animate the gripper transform by one step to the same end-effector position
                
                try 
                    delete(self.cupID{iD}); 
                catch 
                end
                currentPose = self.cupbot.model.getpos(); % initialise a variable with the right number of joint values
                cupPos = self.cupbot.model.fkine(currentPose).T;
                self.cupID{iD} = PlaceObject('Models\cup2.ply', cupPos(1:3, 4)'+ [self.armToCupOffset, 0, 0]);
                drawnow();
                pause(0.01);
            end
        end

        %% UR3 and IRB animation with empty cups
        function animateUR3(self, ur3traj, stepCount)            
            for i = 1:stepCount
                self.cupbot.model.animate(ur3traj(i,:)) % Animate the robot's movement
                self.transformGripper(self); % Animate the gripper transform by one step to the same end-effector position
                drawnow();
                pause(0.01);
            end
        end

        %% UR3 and IRB animation with empty cups
        function animateUR3WithCup(self, ur3traj, stepCount, iD)            
            for i = 1:stepCount
                self.cupbot.model.animate(ur3traj(i,:)) % Animate the robot's movement
                self.transformGripper(self); % Animate the gripper transform by one step to the same end-effector position
                
                try 
                    delete(self.cupID{iD}); 
                catch 
                end
                currentPose = self.cupbot.model.getpos(); % initialise a variable with the right number of joint values
                cupPos = self.cupbot.model.fkine(currentPose).T;
                self.cupID{iD} = PlaceObject('Models\cup2.ply', cupPos(1:3, 4)'+ [self.armToCupOffset, 0, 0]);
                drawnow();
                pause(0.01);
                drawnow();
                pause(0.01);
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

                cupPos = trEnd;
                self.cupID{iD} = PlaceObject('Models\cup2.ply', cupPos(1:3, 4)'+ [self.armToCupOffset, 0, 0]);

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

 %%

        function transformGripper(self)
            % Assuming goalMatrix is the transformation matrix for the end effector pose
            cupbotPose = self.cupbot.model.fkine(self.cupbot.model.getpos()).T;        
            endEffectorYAxis = cupbotPose(1:3, 2);  % Extract the Y-axis
            globalYAxis = [0, 1, 0];
            % Calculate the dot product (angle between the normal vectors)
            dotProduct = dot(endEffectorYAxis, globalYAxis);
            % Calculate the magnitudes (lengths) of the normal vectors
            magnitude1 = norm(endEffectorYAxis);
            magnitude2 = norm(globalYAxis);
            % Calculate the angle between the planes
            angleRadians = acos(dotProduct / (magnitude1 * magnitude2));
                
            % Calculate the desired gripper orientation using the calculated angle
            gripperOrientation = eye(4);
            gripperOrientation(1:3,1:3) = rpy2r(0, angleRadians-pi/2, 0);
            gripperOrientation(1:3, 4) = [0; 0; 0];

            % Animate the gripper to the desired orientation
            startTr1 = self.gripper{1}.model.fkine(self.gripper{1}.model.getpos()).T; % current position of gripper base
            endTr1 = self.cupbot.model.fkine(self.cupbot.model.getpos()).T * trotx(pi/2) * gripperOrientation;
            trPath1 = ctraj(startTr1, endTr1, 2); % Use ctraj to define a trajectory that accounts for translation as well from startTr1 to endTr1
        
            % Calculate the new endTr2 based on endTr1 rotated by troty(pi)
            endTr2 = endTr1 * troty(pi); % goal position of gripper 2 is the same as gripper 1 but rotated about y to mirror the finger
            trPath2 = ctraj(startTr1, endTr2, 2);  % Use ctraj to define a trajectory that accounts for translation as well from startTr1 to endTr2
        
            for i = 1:size(trPath1, 3)
                % Animate gripper{1} from startTr1 to endTr1
                self.gripper{1}.model.base = trPath1(:, :, i);
                self.gripper{1}.model.animate(self.gripper{1}.model.getpos);
                % Animate gripper{2} from startTr1 to endTr2
                self.gripper{2}.model.base = trPath2(:, :, i);
                self.gripper{2}.model.animate(self.gripper{2}.model.getpos);
            end
        end

        % %% Read Arduino Data
        % function receivedData = readDataFromArduino(self)
        %     % Read data from the Arduino
        %     if self.Arduino.BytesAvailable > 0
        %         receivedData = fscanf(self.Arduino, '%d');
        %     else
        %         receivedData = NaN; % Return a default value if no data is available
        %     end
        % end
        % 
        % %% Write to Arduino
        % function sendDataToArduino(self, data)
        %     % Send data to the Arduino
        %     fprintf(self.Arduino, '%d', data);
        % end
        %% 
        function qMatrix = rmrc(self, startQ, endPos, steps, robot)
            T1 = robot.model.fkine(startQ).T;

            M = [1 1 1 zeros(1,3)];                         % Masking Matrix

            x1 = [robot.model.fkine(startQ).t(1) robot.model.fkine(startQ).t(2) robot.model.fkine(startQ).t(3)]';
            x2 = endPos';
            deltaT = 0.05;     % change in time between each step                                   % Discrete time step

            % 3.7
            x = zeros(3,steps); % zeros for columns 1 to steps. Stores x and y and z positions for each step of the trajectory
            s = lspb(0,1,steps);                                 % Create interpolation scalar
            for i = 1:steps
                x(:,i) = x1*(1-s(i)) + s(i)*x2; % x position at each step                 % Create trajectory in x-y plane
            end

            % 3.8
            qMatrix = nan(steps,6); % stores joint angles at each step

            qMatrix(1,:) = robot.model.ikine(T1, 'q0', startQ, 'mask', M);

            % 3.10
            for i = 1:steps-1
                xdot = (x(:,i+1) - x(:,i))/deltaT;   % calculates velocity at each position by getting change between next and current position and dividing by time step                          % Calculate velocity at discrete time step
                xdot = [xdot' 0 0 0];
                J = robot.model.jacob0(qMatrix(i,:));            % Get the Jacobian at the current state
                J = J(1:6,:);                           % Take only first 2 rows
                qdot = inv(J)*xdot'; % change in joint angles   (velocity of joint angles)                         % Solve velocitities via RMRC
                qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';                   % Update next joint state
            end
        end

        
        %% Toppings dispense
        function dispenseToppings(self)
            q = self.irb.model.getpos;
            endPos = self.irb.model.fkine(q);
            totalToppings = 5;
            distance = 0.15;
            increments = 0.02;
            for i = 1:totalToppings
                for j = i:-1:1
                    if increments*(i-j) < distance
                        toppings{j} = PlaceObject('Models\ball3.ply', [endPos.t(1), endPos.t(2), endPos.t(3)-increments*(i-j)]);
                    end
                    try
                        delete(toppings{j-1})
                    end
                end
                pause(0.001);
            end

            for i = totalToppings:-1:1
                try
                    delete(toppings{i})
                end
                pause(0.1);
            end
        end

        %% Toppings pick up
        function pickupToppings(self)
            q = self.irb.model.getpos;
            endPos = self.irb.model.fkine(q);
            totalToppings = 5;
            distance = 0.05;
            increments = 0.02;
            for i = 1:totalToppings
                for j = i:-1:1
                    if increments*(i-j) < distance
                        toppings{j} = PlaceObject('Models\ball3.ply', [endPos.t(1), endPos.t(2), endPos.t(3)-distance+increments*(i-j)]);
                    end
                    try
                        delete(toppings{j-1})
                    end
                end
                pause(0.001);
            end

            for i = totalToppings:-1:1
                try
                    delete(toppings{i})
                end
                pause(0.1);
            end
        end
    end
end