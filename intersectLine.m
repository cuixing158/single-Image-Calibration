function points = intersectLine(line1,line2)
% Brief: two line intersection point on image plane
% Details:
%    None
% 
% Syntax:  
%     points = intersectLine(line1,lne2)
% 
% Inputs:
%    line1 - [1,1] size,[images.roi.Line] type,Description
%    line2 - [1,1] size,[images.roi.Line] type,Description
% 
% Outputs:
%    points - [1,2] size,[double] type,intersection point
% 
% Example: 
%    None
% 
% See also: None

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         11-Aug-2022 17:17:05
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
%
arguments
    line1 (1,1) images.roi.Line
    line2 (1,1) images.roi.Line
end
pts1 = line1.Position;
pts2 = line2.Position;

% line1's 2 points,(x1,y1),(x2,y2), line2's 2 points,(m1,n1),(m2,n2)
x1 = pts1(1,1);y1 = pts1(1,2);
x2 = pts1(2,1);y2 = pts1(2,2);
m1 = pts2(1,1);n1 = pts2(1,2);
m2 = pts2(2,1);n2 = pts2(2,2);

determinant = m1*y1 - n1*x1 - m1*y2 - m2*y1 + n1*x2 + n2*x1 + m2*y2 - n2*x2;
if determinant==0
    points = [inf,inf];
else
    points = [(m1*n2*x1 - m2*n1*x1 - m1*n2*x2 + m2*n1*x2 - m1*x1*y2 + m1*x2*y1 + m2*x1*y2 - m2*x2*y1)./determinant,...
        (m1*n2*y1 - m2*n1*y1 - m1*n2*y2 + m2*n1*y2 - n1*x1*y2 + n1*x2*y1 + n2*x1*y2 - n2*x2*y1)./determinant];
end

end