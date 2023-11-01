% function [] = CollisionTest

close all;
clear; 
clc; 

surf([-2,-2;2,2],[-2,2;-2,2],[-0.605,-0.605;-0.605,-0.605],'CData',imread('Models\concrete.jpg'),'FaceColor','texturemap'); % Generating concrete floor
hold on;
cupLocation = [0, -0.5101, 0.3491];
 

% 2.2 and 2.3
centerpnt = [0.6,-0.6,0];
side = 1;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);

% plyFilePath = 'Models/Obstruction.ply';
% plotOptions.plotFaces = true;
% translationVector = [0.25, -0.5, 0]; % Adjust the translation as needed
% [vertex, faces, faceNormals] = LoadPLYAndVisualize(plyFilePath, plotOptions,translationVector);

axis equal
camlight



%% set up robot

irb = IRB1200;
q = irb.model.getpos;
endJoint = deg2rad([0 -38 5 0 -40 0]);
qWaypoints = [q; endJoint];

steps = 50;

irb.model.teach
trajectory = jtraj(q,endJoint,steps);

for i = 1:steps
    if IsCollision(irb,trajectory(i,:),faces,vertex,faceNormals)
        disp('Collision Detected')
    end
    irb.model.animate(trajectory(i,:)); % Animating the robot to move to the set joint configuration

    drawnow()
    pause(0.05);
end



%% Move to cup - RMRC
steps = 100;


% 3.6
q = irb.model.getpos;
T1 = irb.model.fkine(q).T;       % First pose

% 3.3
M = [1 1 1 zeros(1,3)];                         % Masking Matrix

x1 = [irb.model.fkine(q).t(1) irb.model.fkine(q).t(2) irb.model.fkine(q).t(3)]';
x2 = cupLocation';
deltaT = 0.05;     % change in time between each step                                   % Discrete time step

% 3.7
x = zeros(3,steps); % zeros for columns 1 to steps. Stores x and y and z positions for each step of the trajectory
s = lspb(0,1,steps);                                 % Create interpolation scalar
for i = 1:steps
    x(:,i) = x1*(1-s(i)) + s(i)*x2; % x position at each step                 % Create trajectory in x-y plane
end

% 3.8
trajectory = nan(steps,6); % stores joint angles at each step

% 3.9
% UPDATE: ikine function now has different syntax when entering
% arguments into the function call. The argument must be prefaced by
% its argument name. E.g. Initial Q Guess = 'q0', q & Mask = 'mask', m.
trajectory(1,:) = irb.model.ikine(T1, 'q0', q, 'mask', M);   % sets the inital joint angle              % Solve for joint angles

% 3.10
for i = 1:steps-1
    xdot = (x(:,i+1) - x(:,i))/deltaT;             % calculates velocity at each position by getting change between next and current position and dividing by time step                          % Calculate velocity at discrete time step
    xdot = [xdot' 0 0 0];
    J = irb.model.jacob0(trajectory(i,:));            % Get the Jacobian at the current state
    J = J(1:6,:);                                  % Take only first 2 rows
    qdot = inv(J)*xdot';                           % change in joint angles   (velocity of joint angles)                         % Solve velocitities via RMRC
    trajectory(i+1,:) =  trajectory(i,:) + deltaT*qdot'; % Update next joint state
end

% robot.model.plot(qMatrix,'trail','r-');
collisionThreshold = 0.05; % Joint increment value
updatedTrajectory = trajectory; % Initialize with the original trajectory

for i = 1:steps
    if IsCollision(irb, updatedTrajectory(i,:), faces, vertex, faceNormals)
        % disp('Collision Detected')
        % Incrementally increase joint angles until a non-colliding configuration is found
        while IsCollision(irb, updatedTrajectory(i,:), faces, vertex, faceNormals)
            updatedTrajectory(i, :) = updatedTrajectory(i, :) + collisionThreshold;
        end
    end
    irb.model.animate(updatedTrajectory(i, :)); % Animating the robot to move to the set joint configuration
    drawnow()
    pause(0.05);
end


% fix angle
q = irb.model.getpos;
angleDiff = -90-rad2deg(q(2))-rad2deg(q(3))-rad2deg(q(5));
qEnd = q;
qEnd(5) = qEnd(5) + deg2rad(angleDiff);
qEnd(4) = 0;
trajectory = jtraj(q,qEnd,steps);
for i = 1:steps
    %result(i) = IsCollision(irb,trajectory(i,:),faces,vertex,faceNormals,false);
    if IsCollision(irb,trajectory(i,:),faces,vertex,faceNormals)
        disp('Collision Detected')
        irb.model.animate(coltrajectory(i,:)); % Animating the robot to move to the set joint configuration
    else
    irb.model.animate(trajectory(i,:)); % Animating the robot to move to the set joint configuration 
    end
    drawnow()
    pause(0.05);
end

% end
%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                %plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                %disp('Intersection');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end    
    end
end
end

%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses(q, robot)

links = robot.model.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = robot.model.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
end
end

%% FineInterpolation
% Use results from Q2.6 to keep calling jtraj until all step sizes are
% smaller than a given max steps size
function qMatrix = FineInterpolation(q1,q2,maxStepRadians)
if nargin < 3
    maxStepRadians = deg2rad(1);
end
    
steps = 2;
while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);
end

%% InterpolateWaypointRadians
% Given a set of waypoints, finely intepolate them
function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
if nargin < 2
    maxStepRadians = deg2rad(1);
end

qMatrix = [];
for i = 1: size(waypointRadians,1)-1
    qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
end
end

%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end