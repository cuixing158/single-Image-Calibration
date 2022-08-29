function imagePoints = drawImagePoints(objectLabels,colors)
% Brief: Interactive acquisition of 2D pixel point coordinates of an image 
%      corresponding to true-value 3D world coordinate points
% Details:
%    User clicks on points in the camera image that correspond to 3D
%    coordinates. The image pixel coordinates  are returned as arrays. 
%    Interaction methods are detailed in the build-in `drawpolygon`
%    function, and the order and colours are drawn with reference to
%    "ground truth data" figure. NOTE:The order of the vertices of an
%    object quadrilateral is bottom left, bottom right, top right, top
%    left.  
% 
% Syntax:  
%     imagePoints = drawImagePoints(objectLabels,colors)
% 
% Inputs:
%    objectLabels - [m,1] size,[string] type,Description
%    colors - [m,3] size,[double] type, corresponding to
%    objectLabels,[r,g,b],intensity value,range [0,1].
% 
% Outputs:
%    imagePoints - [m,2] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: build-in `drawpolygon` function

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         11-Aug-2022 15:29:48
% Version history revision notes:
%                                  None
% this function is adapted from:
% Aaron T. Becker's Robot Swarm Lab (2022). Calibrate Camera with one Photo
% (Linear Method)
% (https://www.mathworks.com/matlabcentral/fileexchange/73079-calibrate-camera-with-one-photo-linear-method),
% MATLAB Central File Exchange. Retrieved August 11, 2022.   
%
% Implementation In Matlab R2022a
%
arguments
    objectLabels (:,1) string
    colors (:,3) double
end

% get image points corresponding to 3D world points.
imagePoints = zeros(0,2);% initialization
for i = 1:size(objectLabels,1)
    % draw quadrilateral,bottom left, bottom right, top right, top left
    roi = drawpolygon(Color=colors(i,:),Label=objectLabels(i),FaceAlpha=0.5);
    if size(roi.Position,1)~=4
        error("The "+objectLabels(i)+" polygon must be drawn as a quadrilateral,you should try again.");
    end
    title("confirm quadrilateral position?"+newline+...
        "Adjustment of point/quadrilateral position."+newline+...
    "press any key to confirm and continue!")
    pause; % press any key to continue
    imagePoints = [imagePoints;roi.Position];
end


