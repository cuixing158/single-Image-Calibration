function pointsOrthoCenter = getOrthoCenter(pointA,pointB,pointC)
% Brief: get the orthocenter of triangle ABC from vertices point A/B/C
% Details:
%    None
% 
% Syntax:  
%     pointsOrthoCenter = getOrthoCenter(pointA,pointB,pointC)
% 
% Inputs:
%    pointA - [1,2] size,[double] type,Description
%    pointB - [1,2] size,[double] type,Description
%    pointC - [1,2] size,[double] type,Description
% 
% Outputs:
%    pointsOrthoCenter - [1,2] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         12-Aug-2022 08:38:06
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
%
arguments
    pointA (1,2) double
    pointB (1,2) double
    pointC (1,2) double
end

normBC = [pointC(1,2)-pointB(1,2),pointB(1,1)-pointC(1,1)];
plumbLine1 = images.roi.Line(Position=[pointA;pointA+normBC]);
normAC = [pointC(1,2)-pointA(1,2),pointA(1,1)-pointC(1,1)];
plumbLine2 = images.roi.Line(Position=[pointB;pointB+normAC]);
pointsOrthoCenter = intersectLine(plumbLine1,plumbLine2);
end