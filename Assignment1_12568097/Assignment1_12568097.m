function Final()
clear all
close all
clf

%% Load the environment
[f,v,data] = plyread('Room89.ply','tri');
% Scale the colours to be 0-to-1 (they are originally 0-to-255)
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Plot the environment at origin
tableMesh = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

axis equal 


%% Plot UR5 in environment
robot5 = LinearUR5(false);      % Call for LinearUR5 class file 
hold on

%% Plot UR3 in environment
robot3 = UR3();                 % Call for UR3 class file
hold on

%% little green pop-up table
[f,v,data] = plyread('wall_base.ply','tri');
wallVertexCount = size(v,1);   % Get vertex count                                                          
midPoint = sum(v)/wallVertexCount;     % Move centre point to origin
wallVerts = v - repmat(midPoint, wallVertexCount,1);    % Create a transform to describe the location (at the origin, since it's centered
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;   % Scale the colours to be 0-to-1 (they are originally 0-to-255
% Plot the trisurf
wall_base = trisurf(f,wallVerts(:,1)-0.25,wallVerts(:,2)+-0.65, wallVerts(:,3)+0.4355 ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

%% x,y,z coordinate variables for bricks
zi = 0.9405
x1 = -0.08; y1 = 0;
x2 = -0.08; y2 = 0.28;
x3 = -0.08; y3 = -0.28
x4 = -0.24; y4 = 0;
x5 = -0.24; y5 = 0.28;
x6 = -0.24; y6 = -0.28;
x7 = -0.4; y7 = 0;
x8 = -0.4; y8 = 0.28;
x9 = -0.4; y9 = -0.28;

%% a,b,c coordinates for Donald Trump's wall. Doing my part to make America great again!!11!
ax = 0.2; ay = -0.7;
bx = -0.25; by = -0.7;
cx = -0.5; cy = -0.7;

%% Load Bricks
display('Loading bricks...');
% LoadBrick1
[f,v,data] = plyread('Brick1.ply','tri');
brickVertexCount1 = size(v,1);
midPoint1 = sum(v)/brickVertexCount1;
brickVerts1 = v - repmat(midPoint1, brickVertexCount1,1);
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

brickMesh_1 = trisurf(f,brickVerts1(:,1)+x1,brickVerts1(:,2)+y1, brickVerts1(:,3)+zi ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    

% LoadBrick2
[f,v,data] = plyread('Brick2.ply','tri');
brickVertexCount2 = size(v,1);
midPoint2 = sum(v)/brickVertexCount2;
brickVerts2 = v - repmat(midPoint2, brickVertexCount2,1);
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

brickMesh_2 = trisurf(f,brickVerts2(:,1)+x2,brickVerts2(:,2)+y2, brickVerts2(:,3)+zi ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


% LoadBrick3
[f,v,data] = plyread('Brick3.ply','tri');
brickVertexCount3 = size(v,1);
midPoint3 = sum(v)/brickVertexCount3;
brickVerts3 = v - repmat(midPoint3, brickVertexCount3,1);
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

brickMesh_3 = trisurf(f,brickVerts3(:,1)+x3,brickVerts3(:,2)+y3, brickVerts3(:,3)+zi ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


% LoadBrick4
[f,v,data] = plyread('Brick4.ply','tri');
brickVertexCount4 = size(v,1);
midPoint4 = sum(v)/brickVertexCount4;
brickVerts4 = v - repmat(midPoint4, brickVertexCount4,1);
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

brickMesh_4 = trisurf(f,brickVerts4(:,1)+x4,brickVerts4(:,2)+y4, brickVerts4(:,3)+zi ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


% LoadBrick5
[f,v,data] = plyread('Brick5.ply','tri');
brickVertexCount5 = size(v,1);
midPoint5 = sum(v)/brickVertexCount5;
brickVerts5 = v - repmat(midPoint5, brickVertexCount5,1);
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

brickMesh_5 = trisurf(f,brickVerts5(:,1)+x5,brickVerts5(:,2)+y5, brickVerts5(:,3)+zi ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


% LoadBrick6
[f,v,data] = plyread('Brick6.ply','tri');
brickVertexCount6 = size(v,1);
midPoint6 = sum(v)/brickVertexCount6;
brickVerts6 = v - repmat(midPoint6, brickVertexCount6,1);
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

brickMesh_6 = trisurf(f,brickVerts6(:,1)+x6,brickVerts6(:,2)+y6, brickVerts6(:,3)+zi ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


% LoadBrick7
[f,v,data] = plyread('Brick7.ply','tri');
brickVertexCount7 = size(v,1);
midPoint7 = sum(v)/brickVertexCount7;
brickVerts7 = v - repmat(midPoint7, brickVertexCount7,1);
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

brickMesh_7 = trisurf(f,brickVerts7(:,1)+x7,brickVerts7(:,2)+y7, brickVerts7(:,3)+zi ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


% LoadBrick8
[f,v,data] = plyread('Brick8.ply','tri');
brickVertexCount8 = size(v,1);
midPoint8 = sum(v)/brickVertexCount8;
brickVerts8 = v - repmat(midPoint8, brickVertexCount8,1);
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

brickMesh_8 = trisurf(f,brickVerts8(:,1)+x8,brickVerts8(:,2)+y8, brickVerts8(:,3)+zi ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


% LoadBrick9
[f,v,data] = plyread('Brick9.ply','tri');
brickVertexCount9 = size(v,1);
midPoint9 = sum(v)/brickVertexCount9;
brickVerts9 = v - repmat(midPoint9, brickVertexCount9,1);
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

brickMesh_9 = trisurf(f,brickVerts9(:,1)+x9,brickVerts9(:,2)+y9, brickVerts9(:,3)+zi ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


display('Environment has initialised, press enter to continue');
pause;



%% RobotTR1

steps = 30;
q1 = [0 0 0 0 0 0 0];
q2 = [0 0 0 0 0 0];

% Get current brick coordinates 
ur5BrickPose1 = eye(4)*transl(x1,y1,zi+0.125)*trotx(180, 'deg');    % Variables are offsets to account for gripper height
ur3BrickPose1 = eye(4)*transl(x8,y8,zi+0.04)*trotx(180, 'deg');     % Rotation added to ensure gripper points down

% Move robot to current brick coordinates
ur5_b1= robot5.model.ikcon(ur5BrickPose1, q1);                      % Compute inverse kinematic (quintic) 
ur5Traj5tob1 = jtraj(q1,ur5_b1,steps)                               % Compute joint space trajectory between current location for robot & brick 

ur3_b1= robot3.model.ikcon(ur3BrickPose1, q2);                      
ur3Traj5tob1 = jtraj(q2,ur3_b1,steps)
    
    for i = 1:steps
        animate(robot5.model,ur5Traj5tob1(i,:));                    % Animate UR5 to current brick location
        animate(robot3.model,ur3Traj5tob1(i,:));                    % Animate UR3 to current brick location
        drawnow();
    end

    
ur5forwardTR_1 = makehgtform('translate', [-0.13,0.65,0.125]);                      % Get new brick coordinates
ur5randRotateTR_1 = makehgtform('zrotate', pi/2);                                   % Rotate by 90 degrees (perpendicular) to build wall

ur5BrickPose1 = ur5BrickPose1*ur5forwardTR_1*ur5randRotateTR_1;                     % Multiply new brick pose by translation and rotatoin
ur5UpdatedPoints_1 = [ur5BrickPose1*[brickVerts1,ones(brickVertexCount1,1)]']';     
brickMesh_1.Vertices = ur5UpdatedPoints_1(:,1:3);                                   % Update brick's vertex point
    
  
ur3forwardTR_1 = makehgtform('translate', [-0.1,0.93,0.05]);                        % Now do the same for UR3's bricks
ur3randRotateTR_1 = makehgtform('zrotate', pi/2);

ur3BrickPose1 = ur3BrickPose1*ur3forwardTR_1*ur3randRotateTR_1;
ur3UpdatedPoints_1 = [ur3BrickPose1*[brickVerts8,ones(brickVertexCount8,1)]']';
brickMesh_8.Vertices = ur3UpdatedPoints_1(:,1:3);
  

ur5BrickPose1_T = eye(4)*transl(bx,by,zi+0.125)*trotx(180, 'deg');      % Figure out new brick position on wall - (a,b,c) coordinates
ur3BrickPose1_T = eye(4)*transl(cx,cy,zi+0.04)*trotx(180, 'deg');       % Do the same for UR3

ur5_b1_T= robot5.model.ikcon(ur5BrickPose1_T, q1);                      % Compute inverse kinematics to get there
ur5Traj5tob1_T = jtraj(q1,ur5_b1_T,steps)                               % Then compute joint space trajectory

ur3_b1_T= robot3.model.ikcon(ur3BrickPose1_T, q2);                      % Do the same for UR3
ur3Traj5tob1_T = jtraj(q2,ur3_b1_T,steps)
    for i = 1:steps
        animate(robot3.model,ur3Traj5tob1_T(i,:));                      % Animate UR5 to new brick location
        animate(robot5.model,ur5Traj5tob1_T(i,:));                      % Animate UR3 to new brick location
        drawnow();
    end

 %% RobotTR2
ur5BrickPose2 = eye(4)*transl(x2,y2,zi+0.125)*trotx(180, 'deg');
ur3BrickPose2 = eye(4)*transl(x7,y7,zi+0.04)*trotx(180, 'deg');


ur5_b2= robot5.model.ikcon(ur5BrickPose2, q1);
ur5Traj5tob2 = jtraj(q1,ur5_b2,steps)

ur3_b2= robot3.model.ikcon(ur3BrickPose2, q2);
ur3Traj5tob2 = jtraj(q2,ur3_b2,steps)
    
    for i = 1:steps
        animate(robot5.model,ur5Traj5tob2(i,:));
        animate(robot3.model,ur3Traj5tob2(i,:));
        drawnow();
    end

    
ur5forwardTR_2 = makehgtform('translate', [-0.15,0.93,0.05]);
ur5randRotateTR_2 = makehgtform('zrotate', pi/2);

ur5BrickPose2 = ur5BrickPose2*ur5forwardTR_2*ur5randRotateTR_2;
ur5UpdatedPoints_2 = [ur5BrickPose2*[brickVerts2,ones(brickVertexCount2,1)]']';
brickMesh_2.Vertices = ur5UpdatedPoints_2(:,1:3);
  
  
ur3forwardTR_2 = makehgtform('translate', [-0.1,0.65,-0.038]);
ur3randRotateTR_2 = makehgtform('zrotate', pi/2);

ur3BrickPose2 = ur3BrickPose2*ur3forwardTR_2*ur3randRotateTR_2;
ur3UpdatedPoints_2 = [ur3BrickPose2*[brickVerts7,ones(brickVertexCount7,1)]']';
brickMesh_7.Vertices = ur3UpdatedPoints_2(:,1:3);


ur5BrickPose2_T = eye(4)*transl(bx,by,zi+0.125+0.073)*trotx(180, 'deg');
ur3BrickPose2_T = eye(4)*transl(cx,cy,zi+0.04+0.073)*trotx(180, 'deg');

ur5_b2_T= robot5.model.ikcon(ur5BrickPose2_T, q1);
ur5Traj5tob2_T = jtraj(q1,ur5_b2_T,steps)

ur3_b2_T= robot3.model.ikcon(ur3BrickPose2_T, q2);
ur3Traj5tob2_T = jtraj(q2,ur3_b2_T,steps)
    for i = 1:steps
        animate(robot3.model,ur3Traj5tob2_T(i,:));
        animate(robot5.model,ur5Traj5tob2_T(i,:));
        drawnow();
    end
   
%% RobotTR3
ur5BrickPose3 = eye(4)*transl(x3,y3,zi+0.125)*trotx(180, 'deg');


ur5_b3= robot5.model.ikcon(ur5BrickPose3, q1);
ur5Traj5tob3 = jtraj(q1,ur5_b3,steps)

ur3BrickPose3_T = eye(4)*transl(-0.6,-0.3,zi+0.4+0.073)*trotx(180, 'deg')  
ur3_b3_T= robot3.model.ikcon(ur3BrickPose3_T, q2);
ur3Traj5tob3_T = jtraj(q2,ur3_b3_T,steps)

    for i = 1:steps
        animate(robot3.model,ur3Traj5tob3_T(i,:));
        animate(robot5.model,ur5Traj5tob3(i,:));
        drawnow();
    end

ur5forwardTR_3 = makehgtform('translate', [-0.4,0.375,-0.05]);
ur5randRotateTR_3 = makehgtform('zrotate', pi/2);

ur5BrickPose3 = ur5BrickPose3*ur5forwardTR_3*ur5randRotateTR_3;
ur5UpdatedPoints_3 = [ur5BrickPose3*[brickVerts3,ones(brickVertexCount3,1)]']';
brickMesh_3.Vertices = ur5UpdatedPoints_3(:,1:3);
  
  
ur5BrickPose3_T = eye(4)*transl(cx,cy,zi+0.125+0.073+0.073)*trotx(180, 'deg');
ur5_b3_T= robot5.model.ikcon(ur5BrickPose3_T, q1);
ur5Traj5tob3_T = jtraj(q1,ur5_b3_T,steps)

    for i = 1:steps
        animate(robot5.model,ur5Traj5tob3_T(i,:));
        drawnow();
    end

%% RobotTR4
ur5BrickPose4 = eye(4)*transl(x4,y4,zi+0.125)*trotx(180, 'deg');
ur5_b4= robot5.model.ikcon(ur5BrickPose4, q1);
ur5Traj5tob4 = jtraj(q1,ur5_b4,steps)

    for i = 1:steps       
        animate(robot5.model,ur5Traj5tob4(i,:));
        drawnow();
    end

ur5forwardTR_4 = makehgtform('translate', [0,0.65,-0.05]);
ur5randRotateTR_4 = makehgtform('zrotate', pi/2);

ur5BrickPose4 = ur5BrickPose4*ur5forwardTR_4*ur5randRotateTR_4;
ur5UpdatedPoints_4 = [ur5BrickPose4*[brickVerts4,ones(brickVertexCount4,1)]']';
brickMesh_4.Vertices = ur5UpdatedPoints_4(:,1:3);


ur5BrickPose4_T = eye(4)*transl(bx,by,zi+0.125+0.073+0.073)*trotx(180, 'deg');
ur5_b4_T= robot5.model.ikcon(ur5BrickPose4_T, q1);
ur5Traj5tob4_T = jtraj(q1,ur5_b4_T,steps)

    for i = 1:steps
        animate(robot5.model,ur5Traj5tob4_T(i,:));
        drawnow();
    end
 
%% RobotTR5
ur5BrickPose5 = eye(4)*transl(x5,y5,zi)*trotx(180, 'deg');
ur5_b5= robot5.model.ikcon(ur5BrickPose5, q1);
ur5Traj5tob5 = jtraj(q1,ur5_b5,steps)

    for i = 1:steps       
        animate(robot5.model,ur5Traj5tob5(i,:));
        drawnow();
    end

ur5forwardTR_5 = makehgtform('translate', [0.35,0.925,0]);
ur5randRotateTR_5 = makehgtform('zrotate', pi/2);

ur5BrickPose5 = ur5BrickPose5*ur5forwardTR_5*ur5randRotateTR_5;
ur5UpdatedPoints_5 = [ur5BrickPose5*[brickVerts5,ones(brickVertexCount5,1)]']';
brickMesh_5.Vertices = ur5UpdatedPoints_5(:,1:3);


ur5BrickPose5_T = eye(4)*transl(ax,ay,zi+0.125)*trotx(180, 'deg');
ur5_b5_T= robot5.model.ikcon(ur5BrickPose5_T, q1);
ur5Traj5tob5_T = jtraj(q1,ur5_b5_T,steps)

    for i = 1:steps
        animate(robot5.model,ur5Traj5tob5_T(i,:));
        drawnow();
    end
%% RobotTR6
ur5BrickPose6 = eye(4)*transl(x6,y6,zi+0.125)*trotx(180, 'deg');
ur5_b6= robot5.model.ikcon(ur5BrickPose6, q1);
ur5Traj5tob6 = jtraj(q1,ur5_b6,steps)

    for i = 1:steps       
        animate(robot5.model,ur5Traj5tob6(i,:));
        drawnow();
    end
    
ur5forwardTR_6 = makehgtform('translate', [0.35,0.4,0.05]);
ur5randRotateTR_6 = makehgtform('zrotate', pi/2);

ur5BrickPose6 = ur5BrickPose6*ur5forwardTR_6*ur5randRotateTR_6;
ur5UpdatedPoints_6 = [ur5BrickPose6*[brickVerts6,ones(brickVertexCount6,1)]']';
brickMesh_6.Vertices = ur5UpdatedPoints_6(:,1:3);


ur5BrickPose6_T = eye(4)*transl(ax,ay,zi+0.125+0.073)*trotx(180, 'deg');
ur5_b6_T= robot5.model.ikcon(ur5BrickPose6_T, q1);
ur5Traj5tob6_T = jtraj(q1,ur5_b6_T,steps)

    for i = 1:steps
        animate(robot5.model,ur5Traj5tob6_T(i,:));
        drawnow();
    end
%% RobotTR7
ur5BrickPose7 = eye(4)*transl(x9,y9,zi+0.125)*trotx(180, 'deg');
ur5_b7= robot5.model.ikcon(ur5BrickPose7, q1);
ur5Traj5tob7 = jtraj(q1,ur5_b7,steps)

    for i = 1:steps       
        animate(robot5.model,ur5Traj5tob7(i,:));
        drawnow();
    end

ur5forwardTR_7 = makehgtform('translate', [0.55,0.375,-0.0305]);
ur5randRotateTR_7 = makehgtform('zrotate', pi/2);

ur5BrickPose7 = ur5BrickPose7*ur5forwardTR_7*ur5randRotateTR_7;
ur5UpdatedPoints_7 = [ur5BrickPose7*[brickVerts9,ones(brickVertexCount9,1)]']';
brickMesh_9.Vertices = ur5UpdatedPoints_7(:,1:3);


ur5BrickPose7_T = eye(4)*transl(ax,ay,zi+0.125+0.073+0.073)*trotx(180, 'deg');
ur5_b7_T= robot5.model.ikcon(ur5BrickPose7_T, q1);
ur5Traj5tob7_T = jtraj(q1,ur5_b7_T,steps)

    for i = 1:steps
        animate(robot5.model,ur5Traj5tob7_T(i,:));
        drawnow();
    end


display('Wall created, press enter to plot workspace');
pause;
%% Plot workspace & volume
L1 = Link('d',0.1519,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
L2 = Link('d',0,'a',-0.24365,'alpha',0,'offset',pi/2,'qlim',[deg2rad(-90),deg2rad(90)]);
L3 = Link('d',0,'a',-0.21325,'alpha',0,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'offset',-pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);
L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
L6 = Link('d',0.0819,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);


robot3 = SerialLink([L1 L2 L3 L4 L5 L6], 'base',transl(-0.6, -0.25, 0.904));
    
stepRads = deg2rad(60);         % Decrease angle to ge more accurate results
qlim = robot3.qlim;
% Don't need to worry about joint 6
pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
pointCloud = zeros(pointCloudeSize,3);
counter = 1;
tic

for q1 = qlim(1,1):stepRads:qlim(1,2)
    for q2 = qlim(2,1):stepRads:qlim(2,2)
        for q3 = qlim(3,1):stepRads:qlim(3,2)
            for q4 = qlim(4,1):stepRads:qlim(4,2)
                for q5 = qlim(5,1):stepRads:qlim(5,2)
                    % Don't need to worry about joint 6, just assume it=0
                    q6 = 0;
%                     for q6 = qlim(6,1):stepRads:qlim(6,2)
                        q = [q1,q2,q3,q4,q5,q6];
                        tr = robot3.fkine(q);                        
                        pointCloud(counter,:) = tr(1:3,4)';
                        counter = counter + 1; 
                        if mod(counter/pointCloudeSize * 100,1) == 0
                            display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                        end
%                     end
                end
            end
        end
    end
end

% To only plot UR3 values ABOVE the table (z = 0.904)
for i = 1:pointCloudeSize
    if  pointCloud(i,3) < 0.904
        pointCloud(i,3) = 0.904;
    end 
end

plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');


%% Log (not enough time to figure out lmao)
% L = log4matlab('Final.log');
% L.mlog = {L.DEBUG,'UR3','This is a debug message'};
% L.mlog = {L.WARN,'self = Brick(brickLocationPose)','This is a warning message'};
% 
% L.SetLoggerLevel('myClassName',L.WARN)
% L.mlog = {L.DEBUG,'myClassName','This is a debug message'};
% L.mlog = {L.WARN,'myClassName','This is a warning message'};
% L.mlog = {L.ERROR,'myClassName','This is an error message'};
% 
% 
% try
% a(0);
% catch ME
% L.mlog = {L.DEBUG,'myClassName',['There was an error',L.ExceptionToString(ME)]};
% end
% self.L.mlog = {self.L.DEBUG,mfilename('Final'),[self.L.Me,'a =',num2str(a)]};
% 
% 
% velocity = zeros(steps,6);
% acceleration  = zeros(steps,6);
% for i = 2:steps
%     velocity(i,:) = qMatrix(i,:) - qMatrix(i-1,:);                          % Evaluate relative joint velocity
%     acceleration(i,:) = velocity(i,:) - velocity(i-1,:);                    % Evaluate relative acceleration
% end
% 
% qMatrix = jtraj(q1,q2,steps);
% figure(1)
% Robot3.plot(qMatrix,'trail','r-')                                             % Plot the motion between poses, draw a red line of the end-effector path
% figure(2)
% for i = 1:6
%     subplot(3,2,i)
%     plot(qMatrix(:,i),'k','LineWidth',1)
%     title(['Joint ', num2str(i)])
%     xlabel('Step')
%     ylabel('Joint Angle (rad)')
%     
% end
% 
% figure(3)
% for i = 1:6
%     subplot(3,2,i)
%     plot(velocity(:,i),'k','LineWidth',1)
%     title(['Joint ', num2str(i)])
%     xlabel('Step')
%     ylabel('Joint Velocity')
% end
% 
% figure(4)
% for i = 1:6
%     subplot(3,2,i)
%     plot(acceleration(:,i),'k','LineWidth',1)
%     title(['Joint ', num2str(i)])
%     xlabel('Step')
%     ylabel('Joint Acceleration')
% end

    



end