surf([-2,-2;2,2],[-2,2;-2,2],[-0.605,-0.605;-0.605,-0.605],'CData',imread('Models\concrete.jpg'),'FaceColor','texturemap'); % Generating concrete floor
hold on;
toppingsLocation = [0.62, 0, -0.15];
cupLocation = [-0.464, -0.619, 0.252];

%% set up robot
robot = IRB1200;
q = robot.model.getpos;
endJoint = deg2rad([0 -38 5 0 -40 0]);
steps = 20;
trajectory = jtraj(q,endJoint,steps);
for i = 1:steps
    robot.model.animate(trajectory(i,:)); % Animating the robot to move to the set joint configuratio
    drawnow()
    pause(0.005);
end



%% Move to cup - RMRC
steps = 50;


% 3.6
q = robot.model.getpos;
T1 = robot.model.fkine(q).T;       % First pose

% 3.3
M = [1 1 1 zeros(1,3)];                         % Masking Matrix

x1 = [robot.model.fkine(q).t(1) robot.model.fkine(q).t(2) robot.model.fkine(q).t(3)]';
x2 = cupLocation';
deltaT = 0.05;     % change in time between each step                                   % Discrete time step

% 3.7
x = zeros(3,steps); % zeros for columns 1 to steps. Stores x and y and z positions for each step of the trajectory
s = lspb(0,1,steps);                                 % Create interpolation scalar
for i = 1:steps
    x(:,i) = x1*(1-s(i)) + s(i)*x2; % x position at each step                 % Create trajectory in x-y plane
end

% 3.8
qMatrix = nan(steps,6); % stores joint angles at each step

% 3.9
% UPDATE: ikine function now has different syntax when entering
% arguments into the function call. The argument must be prefaced by
% its argument name. E.g. Initial Q Guess = 'q0', q & Mask = 'mask', m.
qMatrix(1,:) = robot.model.ikine(T1, 'q0', [0 0 0 0 0 0], 'mask', M);   % sets the inital joint angle              % Solve for joint angles

% 3.10
for i = 1:steps-1
    xdot = (x(:,i+1) - x(:,i))/deltaT;   % calculates velocity at each position by getting change between next and current position and dividing by time step                          % Calculate velocity at discrete time step
    xdot = [xdot' 0 0 0];
    J = robot.model.jacob0(qMatrix(i,:));            % Get the Jacobian at the current state
    J = J(1:6,:);                           % Take only first 2 rows
    qdot = inv(J)*xdot'; % change in joint angles   (velocity of joint angles)                         % Solve velocitities via RMRC
    qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';                   % Update next joint state
end

% robot.model.plot(qMatrix,'trail','r-');
for i = 1:steps
    robot.model.animate(qMatrix(i,:)); % Animating the robot to move to the set joint configuratio
    drawnow()
    pause(0.005);
end

% fix angle
q = robot.model.getpos;
angleDiff = -90-rad2deg(q(2))-rad2deg(q(3))-rad2deg(q(5));
qEnd = q;
qEnd(5) = qEnd(5) + deg2rad(angleDiff);
qEnd(4) = 0;
trajectory = jtraj(q,qEnd,steps);
for i = 1:steps
    robot.model.animate(trajectory(i,:)); % Animating the robot to move to the set joint configuratio
    drawnow()
    pause(0.005);
end

%% Move to toppings - RMRC
steps = 50;


% 3.6
q = robot.model.getpos;
T1 = robot.model.fkine(q).T;       % First pose
T2 = [eye(3) [0.62, 0, -0.15]'; zeros(1,3) 1];      % Second pose

% 3.3
M = [1 1 1 zeros(1,3)];                         % Masking Matrix

x1 = [robot.model.fkine(q).t(1) robot.model.fkine(q).t(2) robot.model.fkine(q).t(3)]';
x2 = toppingsLocation';
deltaT = 0.05;     % change in time between each step                                   % Discrete time step

% 3.7
x = zeros(3,steps); % zeros for columns 1 to steps. Stores x and y and z positions for each step of the trajectory
s = lspb(0,1,steps);                                 % Create interpolation scalar
for i = 1:steps
    x(:,i) = x1*(1-s(i)) + s(i)*x2; % x position at each step                 % Create trajectory in x-y plane
end

% 3.8
qMatrix = nan(steps,6); % stores joint angles at each step

% 3.9
% UPDATE: ikine function now has different syntax when entering
% arguments into the function call. The argument must be prefaced by
% its argument name. E.g. Initial Q Guess = 'q0', q & Mask = 'mask', m.
qMatrix(1,:) = robot.model.ikine(T1, 'q0', [0 0 0 0 0 0], 'mask', M);   % sets the inital joint angle              % Solve for joint angles

% 3.10
for i = 1:steps-1
    xdot = (x(:,i+1) - x(:,i))/deltaT;   % calculates velocity at each position by getting change between next and current position and dividing by time step                          % Calculate velocity at discrete time step
    xdot = [xdot' 0 0 0];
    J = robot.model.jacob0(qMatrix(i,:));            % Get the Jacobian at the current state
    J = J(1:6,:);                           % Take only first 2 rows
    qdot = inv(J)*xdot'; % change in joint angles   (velocity of joint angles)                         % Solve velocitities via RMRC
    qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';                   % Update next joint state
end

% robot.model.plot(qMatrix,'trail','r-');
for i = 1:steps
    robot.model.animate(qMatrix(i,:)); % Animating the robot to move to the set joint configuratio
    drawnow()
    pause(0.005);
end

% fix angle
q = robot.model.getpos;
angleDiff = -90-rad2deg(q(2))-rad2deg(q(3))-rad2deg(q(5));
qEnd = q;
qEnd(5) = qEnd(5) + deg2rad(angleDiff);
qEnd(4) = 0;
trajectory = jtraj(q,qEnd,steps);
for i = 1:steps
    robot.model.animate(trajectory(i,:)); % Animating the robot to move to the set joint configuratio
    drawnow()
    pause(0.005);
end



%% Toppings pickup
q = robot.model.getpos;
endPos = robot.model.fkine(q);
totalToppings = 10;
distance = 0.15;
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

%% Toppings dispense
q = robot.model.getpos;
endPos = robot.model.fkine(q);
totalToppings = 10;
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