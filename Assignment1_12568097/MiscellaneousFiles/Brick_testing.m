%% Load the cube created and painted in Blender
[f,v,data] = plyread('Brick.ply','tri');
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

TableMesh = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
% PlaceObject('Room89.ply')
axis equal 
workspace = [-2 2 -2 2 -0.05 2]; 
PlaceObject('Brick.ply', [0, 0, 0.904]);
PlaceObject('Brick.ply',[0, 1, 0.904]);

