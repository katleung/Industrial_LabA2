clear;
close all;
clc;
clf;

hold on
axis equal
view(3);


robot1 = IRB1200(); % create an instance of the IRB1200 class
robot2 = teaUR3();  % create an instance of the teaUR3 class

% Set the pose of the teaUR3 robot to (2, 0, 0)
robot2.model.base = transl(2, 0, 0);

% Call the PointCloud method for IRB1200
[centerX1, centerY1, centerZ1, radiusX1, radiusY1, radiusZ1, workspaceRadius1] = robot1.PointCloud();

% Call the PointCloud method for teaUR3
[centerX2, centerY2, centerZ2, radiusX2, radiusY2, radiusZ2, workspaceRadius2] = robot2.PointCloud();

% Given ellipsoid parameters for IRB1200
ellipsoidCenter1 = [centerX1, centerY1, centerZ1];
ellipsoidRadii1 = [radiusX1, radiusY1, radiusZ1];

% Given ellipsoid parameters for teaUR3
ellipsoidCenter2 = [centerX2, centerY2, centerZ2];
ellipsoidRadii2 = [radiusX2, radiusY2, radiusZ2];

% Generate surface points
%CollidingObject = PlaceObject("Models\personFemaleBusiness.ply", [3,1,0]); 
% Generate surface points
[X, Y] = meshgrid(-5:0.1:5,-5:0.1:5);
Z = X;

% Combine surface points as a point cloud
surfacePoints = [X(:), Y(:), Z(:)];

% Calculate algebraic distance for each point for IRB1200
algebraicDist1 = GetAlgebraicDist(surfacePoints, ellipsoidCenter1, ellipsoidRadii1);

% Calculate algebraic distance for each point for teaUR3
algebraicDist2 = GetAlgebraicDist(surfacePoints, ellipsoidCenter2, ellipsoidRadii2);

% Find points inside the ellipsoid for IRB1200
pointsInsideEllipsoid1 = find(algebraicDist1 < 1);

% Find points inside the ellipsoid for teaUR3
pointsInsideEllipsoid2 = find(algebraicDist2 < 1);

% Find the intersection of points inside both ellipsoids
pointsInsideBothEllipsoids = intersect(pointsInsideEllipsoid1, pointsInsideEllipsoid2);

% Display the number of points inside each ellipsoid
disp(['Number of points inside IRB1200 ellipsoid: ', num2str(size(pointsInsideEllipsoid1, 1))]);
disp(['Number of points inside teaUR3 ellipsoid: ', num2str(size(pointsInsideEllipsoid2, 1))]);

% % Display the number of points inside both ellipsoids
disp(['Number of points inside both ellipsoids: ', num2str(size(pointsInsideBothEllipsoids, 1))]);

if pointsInsideEllipsoid1 || pointsInsideEllipsoid2 > 0
    EStop(); 
end

