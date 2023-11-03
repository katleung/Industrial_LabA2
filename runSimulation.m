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
        cupID = {};                             % Initialise empty cell array to store cup handles (so that they can be deleted later)
        cupFillLoc = [0.4, -0.18, 0.518];     % Set filling location for cup
        irbCupFillLoc = [0.35, -0.19, 0.647];
        cupResetLoc = [0.231, 0.333, 0.453];    % Set reset location for cupbot to avoid hitting the next cup
        armToCupOffset = 0.05; 
        
        closedGrip = deg2rad([-27.6, -14.1,-5.88]);         % Set pose for closed gripper position
        openGrip = deg2rad([-10 -10 0]);                    % Set pose for open gripper position
        
        forcedCollision = true;                   %  Creating force collision object to avoid
        obstructionPosition = [0.5,-0.5,-0.152];       % Position of forced collision

        robotApp;
        Arduino;
    end

    methods (Static)
        %% Constructor - Load Environment
        function self = runSimulation(baseTransform)
            close all
            if nargin > 0  % Check if a base transformation matrix is provided
                self.baseTr = self.baseTr * baseTransform; %apply transformation if provided
            end
            
        % set up arduino comms
            self.Arduino = serialport('COM5', 9600);
            fopen(self.Arduino);

            %load in the robot
            self.robotApp = robotGUIa();
            self.cupbot = self.robotApp.ur3robot;
            self.irb = self.robotApp.irb;
            self.gripper{1} = self.robotApp.ur3Gripper{1};
            self.gripper{2} = self.robotApp.ur3Gripper{2};
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

            %load in environment
            self.LoadEnviro(self);

            %fill cups
            self.fillCups(self);

            %Simulated Sensor Inout Demo
            self.environmentalCollision(self);

        % close arduino
            delete(self.Arduino);
            % clear all;
            % close all;

        end
        
        %% Loading the Environment
         function LoadEnviro(self)
            % load in surface texture for marble floor
            surf([-4,-4;4,4],[-4,4;-4,4],[-0.152,-0.152;-0.152,-0.152],'CData',imread('Models\marble.jpg'),'FaceColor','texturemap');    %Load concrete floor
            hold on;

            % placing the environment models
            PlaceObject('Models\bowl.ply', [0.95,-0.9,0]);
            hold on;
            SafetySystem.ActivateAll; % calling the safety system class to import environment models
            PlaceObject('Models\CoffeeTable2.ply',[0.6, 0.3, -0.152]);
            PlaceObject('Models\CoffeeTable3.ply',[0.95,-0.9, -0.152]);
            PlaceObject('Models\mount3.ply',[0, 0, 0]);
            PlaceObject('Models\mount4.ply',[0.95,-0.323,0.1913-0.152]);
            
         end

        %% Generating Cup / Cups
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

        %% Move Cup (build wall)
        function fillCups(self)
            % Set the initial joint angles to the current robot position
            q = self.cupbot.model.getpos();
            offsetPose = transl(-self.armToCupOffset, 0, 0)* trotx(pi);
            offsetWaypoint = transl(-self.armToCupOffset, 0, 0.2)* trotx(pi);
            [m,n] = size(self.cupID);
            for i = 1:n
                self.cupStatus(i) = 0;
                while self.cupStatus(i) <4
                    if self.cupStatus(i) == 0 % move to cup start location, close gripper
                        totalSteps = 40;
                        ur3FirstSteps = 30;
                        ur3SecondSteps = totalSteps-ur3FirstSteps;
                        self.cupbot.model.getpos
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
                        startCupLocationWaypoint = transl(self.cupStartLoc(i,:))*offsetWaypoint;
                        trajectoryAboveCup = self.rmrc(self, self.cupbot.model.getpos, [startCupLocationWaypoint(1,4), startCupLocationWaypoint(2,4), startCupLocationWaypoint(3,4)], ur3SecondSteps, self.cupbot);
                        % move to fill location
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

            % Setting the GUI parameters to enable teach functionality once the simulation has finished
            self.robotApp.simFinished = true;
            self.robotApp.UR3Joint1Slider.Enable = true;
            self.robotApp.UR3Joint2Slider.Enable = true;
            self.robotApp.UR3Joint3Slider.Enable = true;
            self.robotApp.UR3Joint4Slider.Enable = true;
            self.robotApp.UR3Joint5Slider.Enable = true;
            self.robotApp.UR3Joint6Slider.Enable = true;
            self.robotApp.UR3MOVEButton.Enable = true;
            self.robotApp.IRBJoint1Slider.Enable = true;
            self.robotApp.IRBJoint2Slider.Enable = true;
            self.robotApp.IRBJoint3Slider.Enable = true;
            self.robotApp.IRBJoint4Slider.Enable = true;
            self.robotApp.IRBJoint5Slider.Enable = true;
            self.robotApp.IRBJoint6Slider.Enable = true;
            self.robotApp.IRBMOVEButton.Enable = true;
        end

        

        %% UR3 and IRB animation without cup
        function animateBothRobotsNoCup(self, ur3traj, irbTraj, stepCount)            
            i = 1;
            while i <= stepCount % checks if all the steps have been animated
                if strcmp(self.robotApp.systemState, 'running') % checks if the system is estopped
                    updatedirbTraj = irbTraj; % Initialize with the original trajectory
                    if self.forcedCollision == true
                        PlaceObject( 'Models/Obstruction.ply',self.obstructionPosition);
                        plotOptions.plotFaces = false;
                        [vertex, faces, faceNormals] = LoadPLYAndVisualize(self.obstructionPosition,plotOptions);
                        collisionThreshold = 0.05; % Joint increment value
                        if IsCollision(self.irb,updatedirbTraj(i,:),faces,vertex,faceNormals)
                            disp('IRB1200: Collision Detected')
                            while IsCollision(self.irb, updatedirbTraj(i,:), faces, vertex, faceNormals)
                                updatedirbTraj(i, :) = updatedirbTraj(i, :) + collisionThreshold;
                            end
                            disp('Collision Avoided')
                        end
                    end
                    self.irb.model.animate(updatedirbTraj(i,:))
                    if self.forcedCollision == true
                        if IsCollision(self.cupbot,ur3traj(i,:),faces,vertex,faceNormals)
                            disp('UR3: Collision Detected')
                            guiState = self.robotApp.guiEstopStatus;
                            strcmp(guiState, 'eStopped');
                            self.robotApp.updateEStop();
                        end
                    end
                    self.cupbot.model.animate(ur3traj(i,:)) % Animate the robot's movement
                    self.transformGripper(self); % Animate the gripper transform by one step to the same end-effector position
                    self.updateUR3GUI(self); % updates GUI parameters
                    self.updateIRBGUI(self); % updates GUI parameters
                    self.checkArduinoState(self); % check arduino inputs
                    drawnow();
                    pause(0.01);
                    i = i+1;
                else
                    self.checkArduinoState(self); % check arduino input
                    pause(0.01);
                end
            end
        end

        %% UR3 and IRB animation with empty cups
        function animateBothRobotsEmptyCup(self, ur3traj, irbTraj, stepCount, iD)            
            i = 1;
            while i <= stepCount % checks if all the steps have been animated
                if strcmp(self.robotApp.systemState, 'running') % checks if the system is estopped
                    updatedirbTraj = irbTraj; % Initialize with the original trajectory
                    if self.forcedCollision == true
                        PlaceObject( 'Models/Obstruction.ply', self.obstructionPosition);
                        plotOptions.plotFaces = false;
                        [vertex, faces, faceNormals] = LoadPLYAndVisualize(self.obstructionPosition,plotOptions);
                        collisionThreshold = 0.05; % Joint increment value
                    if IsCollision(self.irb,updatedirbTraj(i,:),faces,vertex,faceNormals)
                        disp('IRB1200: Collision Detected')
                        while IsCollision(self.irb, updatedirbTraj(i,:), faces, vertex, faceNormals)
                            updatedirbTraj(i, :) = updatedirbTraj(i, :) + collisionThreshold;
                        end
                        disp('Collision Avoided')
                    end
                    end
                    
                    self.irb.model.animate(updatedirbTraj(i,:))
                    if self.forcedCollision == true
                        if IsCollision(self.cupbot,ur3traj(i,:),faces,vertex,faceNormals)
                            disp('UR3: Collision Detected')
                            guiState = self.robotApp.guiEstopStatus;
                            strcmp(guiState, 'eStopped');
                            self.robotApp.updateEStop();
                        end
                    end
                    self.cupbot.model.animate(ur3traj(i,:)) % Animate the robot's movement
                    self.transformGripper(self); % Animate the gripper transform by one step to the same end-effector position

                    try
                        delete(self.cupID{iD});
                    catch
                    end
                    currentPose = self.cupbot.model.getpos(); % initialise a variable with the right number of joint values
                    cupPos = self.cupbot.model.fkine(currentPose).T; % UR3's current end effector pose
                    self.cupID{iD} = PlaceObject('Models\cup2.ply', cupPos(1:3, 4)'+ [self.armToCupOffset, 0, 0]); % places the cup model at the end effector's location
                    self.updateUR3GUI(self); % updates GUI parameters
                    self.updateIRBGUI(self); % updates GUI parameters
                    self.checkArduinoState(self); % check arduino input
                    drawnow();
                    pause(0.01);
                    i = i+1;
                else
                    self.checkArduinoState(self); % check arduino input
                    pause(0.01);
                end
            end
        end

        %% UR3 animation without cup
        function animateUR3(self, ur3traj, stepCount)
            i = 1;
            while i <= stepCount % checks if all the steps have been animated
                if strcmp(self.robotApp.systemState, 'running') % checks if the system is estopped
                    if self.forcedCollision == true
                        PlaceObject( 'Models/Obstruction.ply',self.obstructionPosition);
                        plotOptions.plotFaces = false;
                        [vertex, faces, faceNormals] = LoadPLYAndVisualize(self.obstructionPosition,plotOptions);
                        if IsCollision(self.cupbot,ur3traj(i,:),faces,vertex,faceNormals)
                            disp('UR3: Collision Detected')
                            guiState = self.robotApp.guiEstopStatus;
                            strcmp(guiState, 'eStopped');
                            self.robotApp.updateEStop();
                        end
                    end
                    self.cupbot.model.animate(ur3traj(i,:)) % Animate the robot's movement
                    self.transformGripper(self); % Animate the gripper transform by one step to the same end-effector position
                    self.updateUR3GUI(self); % updates GUI parameters
                    self.updateIRBGUI(self); % updates GUI parameters
                    self.checkArduinoState(self); % check arduino input
                    drawnow();
                    pause(0.01);
                    i = i+1;
                else
                    self.checkArduinoState(self); % check arduino input
                    pause(0.01);
                end
            end
        end

        %% UR3 animation with empty cups
        function animateUR3WithCup(self, ur3traj, stepCount, iD)
            i = 1;
            while i <= stepCount % checks if all the steps have been animated
                if strcmp(self.robotApp.systemState, 'running') % checks if the system is estopped
                    if self.forcedCollision == true
                        PlaceObject( 'Models/Obstruction.ply',self.obstructionPosition);
                        plotOptions.plotFaces = false;
                        [vertex, faces, faceNormals] = LoadPLYAndVisualize(self.obstructionPosition,plotOptions);
                        if IsCollision(self.cupbot,ur3traj(i,:),faces,vertex,faceNormals)
                            disp('UR3: Collision Detected')
                            guiState = self.robotApp.guiEstopStatus;
                            strcmp(guiState, 'eStopped');
                            self.robotApp.updateEStop();
                        end
                    end
                    self.cupbot.model.animate(ur3traj(i,:)) % Animate the robot's movement
                    self.transformGripper(self); % Animate the gripper transform by one step to the same end-effector position

                    try
                        delete(self.cupID{iD});
                    catch
                    end
                    currentPose = self.cupbot.model.getpos(); % initialise a variable with the right number of joint values
                    cupPos = self.cupbot.model.fkine(currentPose).T; % UR3's current end effector pose
                    self.cupID{iD} = PlaceObject('Models\cup2.ply', cupPos(1:3, 4)'+ [self.armToCupOffset, 0, 0]); % places the cup model at the end effector's location
                    self.updateUR3GUI(self); % updates GUI parameters
                    self.updateIRBGUI(self); % updates GUI parameters
                    self.checkArduinoState(self); % check arduino input
                    drawnow();
                    pause(0.01);
                    i = i+1;
                else
                    self.checkArduinoState(self); % check arduino input
                    pause(0.01);
                end
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

 %% Move gripper position

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

        %% Read Arduino Data
        function receivedData = readDataFromArduino(self)
            % Attempt to read data from the Arduino
            try
                receivedData = fscanf(self.Arduino, '%d');
            catch
                % fprintf('No data from Arduino.\n');
            end
        end

        %% Write to Arduino
        function sendDataToArduino(self, data)
            % Send data (double) to the Arduino
            fprintf(self.Arduino, '%d', data);
        end

        %% EStop State Check
        function checkArduinoState(self)
            guiState = self.robotApp.guiEstopStatus;
            guiState = 'unlinked'; % comment this line out if you want to link arduino estop to gui estop
            if strcmp(guiState, 'eStopped') % if gui release button has been pressed
                self.sendDataToArduino(self, 4); % set arduino to 1 = estop state
            end

            flush(self.Arduino);            
            state = self.readDataFromArduino(self)
            if state == 1 % e-Stop button has been pressed on Arduino
                if strcmp(guiState, 'released') % if gui release button has been pressed
                    self.sendDataToArduino(self, 5); % set arduino to 2 = released state
                end
                self.robotApp.updateEStop(); % activate estop function
            elseif state == 2 % release button has been pressed on arduino
                if strcmp(guiState, 'confirmed') % if gui confirm button has been pressed 
                    self.sendDataToArduino(self, 6); % reset arduino state to 3 = confirm state
                end
                self.robotApp.updateEStopRelease(); % activate release function
            elseif state == 3 % confirm button has been pressed on arduino
                self.sendDataToArduino(self, 7); % reset arduino state to 0 = running
                self.robotApp.updateEStopConfirmed(); % activate confirmed function
            end
            % pause(0.05);
        end

        %% RMRC function
        function qMatrix = rmrc(self, startQ, endPos, steps, robot)
            T1 = robot.model.fkine(startQ).T; % robot's starting pose

            M = [1 1 1 zeros(1,3)]; % Masking Matrix

            x1 = [robot.model.fkine(startQ).t(1) robot.model.fkine(startQ).t(2) robot.model.fkine(startQ).t(3)]'; % robot's start position
            x2 = endPos'; % robot's end position
            deltaT = 0.05; % change in time between each step 

            x = zeros(3,steps); % zeros for columns 1 to steps. Stores x and y and z positions for each step of the trajectory
            s = lspb(0,1,steps); % create interpolation scalar
            for i = 1:steps
                x(:,i) = x1*(1-s(i)) + s(i)*x2; % create robot trajectory
            end

            qMatrix = nan(steps,6); % empty matrix used to store joint angles at each step

            qMatrix(1,:) = robot.model.ikine(T1, 'q0', startQ, 'mask', M); % matrix stores joint angles at each step

            for i = 1:steps-1
                xdot = (x(:,i+1) - x(:,i))/deltaT; % calculates velocity at each position by getting change between next and current position and dividing by time step                          % Calculate velocity at discrete time step
                xdot = [xdot' 0 0 0];
                J = robot.model.jacob0(qMatrix(i,:)); % get the Jacobian at the current state
                J = J(1:6,:); % take the first six rows
                qdot = inv(J)*xdot'; % change in joint angles (velocity of joint angles). Velocities solved via RMRC
                qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot'; % next joint state updates
            end
        end
        
        %% Toppings dispense
        function dispenseToppings(self)
            q = self.irb.model.getpos;
            endPos = self.irb.model.fkine(q);
            totalToppings = 5;
            distance = 0.15;
            increments = 0.02;

            % the for loop goes through all the topics and places an object starting from the robot's end effector position down into the cup
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

            % the for loop goes through all the topics and deletes the objects
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

            % the for loop goes through all the topics and places an object starting from the bowl to the robot's end effector position
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
            
            % the for loop goes through all the topics and deletes the objects
            for i = totalToppings:-1:1
                try
                    delete(toppings{i})
                end
                pause(0.1);
            end
        end

        %% Update GUI UR3
        function updateUR3GUI(self)
            % Update joint values
            q = self.cupbot.model.getpos;
            for i = 1:6
                self.robotApp.qUR3(i) = q(i);
            end
            
            % Update xyzrpy display
            self.robotApp.UpdateUR3Pose();
        end

        %% Update GUI IRB
        function updateIRBGUI(self)
            % Update joint values
            q = self.irb.model.getpos;
            for i = 1:6
                self.robotApp.qIRB(i) = q(i);
            end
            
            % Update xyzrpy display
            self.robotApp.UpdateIRBPose();
        end
          %% Simulated Sensor Enacting Collision
        function environmentalCollision(self)
            planeNormal = [0,1,0];
            planePoint = [0,-1.1,0];

            objectPathFile = 'Models/Obstruction.ply';
            startPosition = [-1,-2,0];
            movementVector = [0,0.1,0];
            sensedObject = PlaceObject(objectPathFile, startPosition);
            steps = 20;

            for i = 1:steps
                delete(sensedObject)
                newPosition = startPosition + i* movementVector;
                % intersection between the line (line) and plane (obstacle)
                [intersectionPoints,check] = LinePlaneIntersection(planeNormal,planePoint,startPosition,newPosition);
                if check == 1
                    disp('Light Gate Detected Motion');
                    guiState = self.robotApp.guiEstopStatus;
                    strcmp(guiState, 'eStopped');
                    self.robotApp.updateEStop();
                    break; 

                end
                sensedObject = PlaceObject(objectPathFile, newPosition);
                drawnow;
                pause(0.5)
            end
        end
    end
end