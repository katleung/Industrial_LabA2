%% Collision Detection Test IRB1200
clear;
close all;
clc;
clf;


hold on
axis equal
view(3);

surf([-4,-4;4,4],[-4,4;-4,4],[-0,-0;-0,-0],'CData',imread('Models\concrete.jpg'),'FaceColor','texturemap'); % Generating concrete floor
SafetySystem.ActivateAll; % Generating Environment

robot1 = IRB1200(); % create an instance of the IRB1200 class

% Call the PointCloud method
[centerX, centerY, centerZ,radiusX, radiusY, radiusZ, workspaceRadius] = robot1.PointCloud();

% Now you can use centerX, centerY, centerZ, and workspaceRadius as needed
disp(['IRB1200 Properties Center: (', num2str(centerX), ', ', num2str(centerY), ', ', num2str(centerZ), ')']);
disp(['Workspace Radius: (', num2str(radiusX), ', ', num2str(radiusY), ', ', num2str(radiusZ), ')', num2str(workspaceRadius)]);

% Given ellipsoid parameters
ellipsoidCenter = [centerX, centerY, centerZ];
ellipsoidRadii = [radiusX, radiusY, radiusZ];

[X,Y,Z] = ellipsoid(centerX, centerY, centerZ,radiusX, radiusY, radiusZ);

ellipsoidAtIRB1200= surf(X,Y,Z);
% Make the ellipsoid translucent (so we can see the inside and outside points)
alpha(0.1);

% Load PLY file
plyData = plyread('Models\personFemaleBusiness.ply');
vertices = [plyData.vertex.x, plyData.vertex.y, plyData.vertex.z];

% Apply transformations (adjust as needed)
transVector = [-4, 0, 0];  % Example translation vector
rotMatrix = eye(3);  % No rotation for now, you can replace with your rotation matrix

% Apply rotation and translation to vertices
transformedVertices = vertices + transVector;

% Combine surface points as a point cloud
cubePoints = transformedVertices;

% Plot the cube or do further processing as needed
cube_h = plot3(cubePoints(:,1), cubePoints(:,2), cubePoints(:,3), 'b.');


% [Y,Z] = meshgrid(-0.75:0.05:0.75,-0.75:0.05:0.75);
% sizeMat = size(Y);
% X = repmat(0.75,sizeMat(1),sizeMat(2));
% oneSideOfCube_h = surf(X,Y,Z);
% 
% % Combine surface points as a point cloud
% cubePoints = [X(:),Y(:),Z(:)];
% cubePoints = [ cubePoints ...
%              ; cubePoints * rotz(pi/2)...
%              ; cubePoints * rotz(pi) ...
%              ; cubePoints * rotz(3*pi/2) ...
%              ; cubePoints * roty(pi/2) ...
%              ; cubePoints * roty(-pi/2)]; 
% 
% cubePoints = cubePoints + repmat([1,0,-0.5],size(cubePoints,1),1);

% Calculate algebraic distance for each point
algebraicDist = GetAlgebraicDist(cubePoints, ellipsoidCenter, ellipsoidRadii);

% Find points inside the ellipsoid
pointsInsideEllipsoid = find(algebraicDist < 1);

% Display the number of points inside the ellipsoid
disp(['Number of points inside the ellipsoid: ', num2str(size(pointsInsideEllipsoid, 1))]);

if pointsInsideEllipsoid > 0
    fprintf('An object has collided with the IRB1200 Workspace. The Estop has been engaged. Please action following request.\n')
    SimulatedEStop();
end

print('There is no collision detect simulation will continue.')





