%% THIS CODE WAS DEVELOPED BY KATRINA LEUNG (13570181)
classdef runSimulation < handle
    properties
        baseTr = eye(4);
        cupbot;                             % Create a robot object
        cupStartLoc = zeros(1, 3);                  % Initialise all cup startLocation with zeros
        brickStatus = zeros(1,1);           % Initialise all cup statuses to 0 (empty)
        cupEndLoc = zeros(1, 3);          % Initialise all cup endLocations with zeros
        cupID = {};                         % Initialise empty cell array to store cup handles (so that they can be deleted later)

        L = log4matlab('runSimulationLog.log');             % Create a log file for debugging

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
            
            %load bricks
            [self.cupID, self.cupStartLoc] = self.loadCups(self); % For testing within the class, without starterScript

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
            cupStartLoc(1,:) = [-0.4, 0.4, 0];
            for n = 1:height(cupStartLoc) % for number of rows in cup array
                updateLoc = transl(cupStartLoc(n,:)) * self.baseTr;
                cupStartLoc(n,:) = [updateLoc(1,4), updateLoc(2,4), updateLoc(3,4)];
                cupID{n} = PlaceObject('HalfSizedRedGreenBrick.ply', cupStartLoc(n,:));
            end 
        end

        %% Move Cup (build wall)


    end
end