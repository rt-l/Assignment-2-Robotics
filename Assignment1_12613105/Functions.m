%% Assignment 1

spawnEnvironment();
spawnRobots();

function[] = spawnEnvironment()

clear all
close all
clc
hold on

axis([-3 3 -3 3 0 3]);
surf([-3,-3;3,3],[-3,3;-3,3],[0.01,0.01;0.01,0.01],'CData',imread('hardwood-floor.jpeg'),'FaceColor','texturemap'); % Concrete floor
surf([-3,3;-3,3],[3,3;3,3],[0,0;3,3],'CData',imread('office_wall_1.jpeg'),'FaceColor','texturemap'); % Concrete floor
surf([3,3;3,3],[-3,3;-3,3],[0,0;3,3],'CData',imread('window.jpeg'),'FaceColor','texturemap'); % Concrete floor

PlaceObject('table.ply',[0 0 0]);
PlaceObject('dummy.ply',[1.5 2 0]);
PlaceObject('chair.ply',[-2 -2 0]);
PlaceObject('chair.ply',[0 -2 0]);
PlaceObject('chair.ply',[2 -2 0]);
PlaceObject('fence1.ply',[-2 0 0]);
PlaceObject('fence1.ply',[2 0 0]);
PlaceObject('fence2.ply',[0 1.2 0]);
PlaceObject('fence2.ply',[0 -1.2 0]);
PlaceObject('firstaid.ply',[-2 2.5 0]);
PlaceObject('stop.ply',[2.7 2.9 1.5]);

axis equal
camlight

%% Brick Initial Placement
% first layer
brick1 = PlaceObject('brick.ply',[0,-0.3,0.2])
brick2 = PlaceObject('brick.ply',[0,0,0.2])
brick3 = PlaceObject('brick.ply',[0,0.3,0.2])
% second layer
brick4 = PlaceObject('brick.ply',[-0.3,-0.3,0.2])
brick5 = PlaceObject('brick.ply',[-0.3,0,0.2])
brick6 = PlaceObject('brick.ply',[-0.3,0.3,0.2])
%third layer
brick7 = PlaceObject('brick.ply',[0.3,-0.3,0.2])
brick8 = PlaceObject('brick.ply',[0.3,0,0.2])
brick9 = PlaceObject('brick.ply',[0.3,0.3,0.2])
end

function[] = spawnRobots()

% LinearUR5
r1 = LinearUR5(false)
hold on;
r1.model.animate(r1.model.getpos)

UR5Pose = [1,0.2,-0.704]
%r1.model.base = trotx(pi/2)*transl(UR5Pose);

r1.model.base = transl(-0.6,-0.25,0.904)

r1.model.base(1:3,4) = [0,0,0]


% UR3
r2 = UR3()
hold on
UR3Pose = [0,0.2,-0.75]


r2.model.animate(r2.model.getpos)
r2.model.base = trotx(pi/2)*transl(UR3Pose);
end









   
   
   
 
   
   
   
   
 
 