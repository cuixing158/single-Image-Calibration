function [f,XYZcoordinates,objectLabels,colors] = plotGroundTruth3D(...
    worldPointsGroundTruth)
% Brief: plot your own image ground truth data
% Note:
%    You should prepare your own 3D data for your single image.
% 
% Syntax:  
%     [f,XYZcoordinates,objectLabels,colors] = plotGroundTruth3D(worldPointsGroundTruth)
% 
% Inputs:
%    worldPointsGroundTruth - [m,4] size,[cell array] type,(optional),your
%    own image ground truth data coordinates in world coordinates and labels.
% 
% Outputs:
%    f - [1,1] size,figure object
%    XYZcoordinates - [m,3] size,[double] type,x,y,z coordinates in world
%    objectLabels - [m,1] size,[string] type,object labels
%    colors - [m,3] size,[double] type,each object [r,g,b] value
% 
% Example: 
%    None
% 
% See also: None

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         11-Aug-2022 16:22:28
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
%

arguments
    worldPointsGroundTruth (:,4) cell  = [... % x, y, z in world coordinates, a text description. All distances are in cm. Designed to work with 4-sided shapes
    % TODO 1: for your picture, you need to generate these yourself
    0,0,-.001, {'table, bottom left'};
    100,0,-.001, {'table, bottom right'};
    100,240,-.001, {'table, top right'};
    0,240,-.001, {'table, top left'};


    0,0,0, {'paper 1, bottom left'};
    28,0,0, {'paper 1, bottom right'};
    28,22,0, {'paper 1, top right'};
    0,22,0, {'paper 1, top left'};

    51,37,2.5, {'book top, bottom left'};
    75,37,2.5, {'book top, bottom right'};
    74,56,2.5, {'book top, top right'};
    50,56,2.5, {'book top, top left'};


    7,58,0, {'paper 2, bottom left'};
    29,58,0, {'paper 2 bottom right'};
    29,86,0, {'paper 2, top right'};
    7,86,0, {'paper 2, top left'};

    78.5,56,0, {'paper 3, bottom left'};
    100,56,0, {'paper 3, bottom right'};
    100,84,0, {'paper 3, top right'};
    78.5,84,0, {'paper 3, top left'};


    78.5,108.5,0, {'paper 4, bottom left'};
    100,108.5,0, {'paper 4, bottom right'};
    100,136,0, {'paper 4, top right'};
    78.5,136,0, {'paper 4, top left'};

    32,83,30, {'cube top, bottom left'};
    63,83,30, {'cube top, bottom right'};
    63,113,30, {'cube top, top right'};
    32,113,30, {'cube top, top left'};

    32,83,0, {'cube front, bottom left'};
    63,83,0, {'cube front, bottom right'};
    63,83,30, {'cube front, top right'};
    32,83,30, {'cube front, top left'};

    63,83,0, {'cube right, bottom left'};
    63,113,0, {'cube right, bottom right'};
    63,113,30, {'cube right, top right'};
    63,83,30, {'cube right, top left'};

    74,150,0, {'tiny cube front, bottom left'};
    80.5,150,0, {'tiny cube front, bottom right'};
    80.5,150,6.5, {'tinycube front, top right'};
    74,150,6.5, {'tinycube front, top left'};


    13,190,0, {'pancake box front, bottom left'};
    30,190,0, {'pancake box front, bottom right'};
    30,190,26, {'pancake box front, top right'};
    13,190,26, {'pancake box front, top left'};

    56,192.5,0, {'tea box front, bottom left'};
    76,192.5,0, {'tea box front, bottom right'};
    76,192.5,29, {'tea box front, top right'};
    56,192.5,29, {'tea box front, top left'};

    -30,55,0, {'cake box front, bottom left'};
    -30,69,0, {'cake box front, bottom right'};
    -30,69,18, {'cake box front, top right'};
    -30,55,18, {'cake box front, top left'};
    ];
end

numObject = size(worldPointsGroundTruth,1)/4;
XYZcoordinates = cell2mat(worldPointsGroundTruth(:,1:3));
allLabels = string(worldPointsGroundTruth(:,4));
objectLabels = allLabels(1:4:end);
objectLabels = extractBefore(objectLabels,",");

%% refactor data type
bottomLeftWorldPoints = XYZcoordinates(1:4:end,:);
bottomRightWorldPoints = XYZcoordinates(2:4:end,:);
topRightWorldPoints = XYZcoordinates(3:4:end,:);
topLeftWorldPoints = XYZcoordinates(4:4:end,:);
pointsSetTable = table(objectLabels,bottomLeftWorldPoints,bottomRightWorldPoints,...
    topLeftWorldPoints,topRightWorldPoints);

% draw 3D world
f = figure(Name="ground truth data");
colors = rand(numObject,3);
for j = 1:numObject
    currPatchTable = pointsSetTable(j,:);
    XYZpatch = [currPatchTable.bottomLeftWorldPoints;
        currPatchTable.bottomRightWorldPoints;
        currPatchTable.topRightWorldPoints;
        currPatchTable.topLeftWorldPoints]; % note the order
    label = objectLabels(j);

    % plot groundTruth 3D shape
    patch(XYZpatch(:,1),XYZpatch(:,2),XYZpatch(:,3),colors(j,:),FaceAlpha=0.5);
    text( mean(XYZpatch(:,1)),mean(XYZpatch(:,2)),mean(XYZpatch(:,3)),label,HorizontalAlignment="center");
end

% plot world coordinate system
axis equal;grid on;hold on;
quiver3(0,0,0,50,0,0,Color='red',LineWidth=2);
quiver3(0,0,0,0,50,0,Color='green',LineWidth=2);
quiver3(0,0,0,0,0,50,Color='blue',LineWidth=2);

xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
view(45,35); % azimuth,elevation angle
