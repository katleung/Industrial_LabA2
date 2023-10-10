close all

% transformation limits for the base of the Linear UR3
% if the base is transformed and remains on the table, the environment will not be moved as well
% if the base is transformed off the table, the location of the table, warn light and e-stop will move accordingly
    % NOTE: the fence, light curtain and fire-extinguisher will not update
% transformation limits to remain on the table are: 
    % [0, 0.8], [-0.5, 0.3], [0,0]

% RUN CODE - ENVIRONMENT SETUP
baseTr = eye(4) * transl(0.5,0,0);                % base pose of LinearUR3
sim = runSimulation(baseTr);                    % Load the environment + robot (Constructor)

%% RUN CODE - BUILD WALL
% close all figures and rerun 'Environment Setup' section for cleaner graphic
[sim.brickID, sim.brickLoc] = runSimulation.loadBricks(sim);
%%
runSimulation.buildWall(sim);

%% RUN CODE - FIND POINT CLOUD (recommended 60-90 deg increments)
[sim.reachRange, sim.reachRadius, sim.reachVolume] = runSimulation.pointCloudPlot(sim,60);

% Print values to terminal
range = sim.reachRange
radius = sim.reachRadius
volume = sim.reachVolume
