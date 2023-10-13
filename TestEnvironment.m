
clear; 
close all; 
clc; 
clf; 

hold on; 
axis equal; 

% PlaceObject('fenceAssemblyGreenRectangle4x8x2.5m.ply',[0,0,0])
Robot = UR3; 

Fence(eye(4)); 
EStop(transl(1.2,-1, 1)); 
EStop(transl(1.2,1.15,1));
FireExtingusher(transl(1,1,0)); 
LightCurtain(transl(2,1,0.75)); 