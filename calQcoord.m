function Q = calQcoord(l,P,M,N)
% Brief: 计算空间平面内满足条件的交点Q
% Details:
%    详见数学分析calExtrinsicAnalysis.mlx中几何求解平移向量小节
% 
% Syntax:  
%     calQcoord(l,P,M,N)
% 
% Inputs:
%    l - [m,n] size,[double] type,Description
%    P - [m,n] size,[double] type,Description
%    M - [m,n] size,[double] type,Description
%    N - [m,n] size,[double] type,Description
% 
% Outputs:
%    Q - [1,3] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         26-Aug-2022 17:35:05
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
%

l1 = l(1,1);l2 = l(1,2);l3 = l(1,3);
x0 = P(1,1);y0 = P(1,2);z0 = P(1,3);
x1 = M(1,1);y1 = M(1,2);z1 = M(1,3);
x2 = N(1,1);y2 = N(1,2);z2 = N(1,3);
Q = [(l2*x0*x1 - l2*x1*x2 - l1*x0*y1 + l1*x0*y2 - l1*x2*y0 + l1*x2*y1)/(l2*x0 - l2*x2 - l1*y0 + l1*y2), (l2*x1*y0 + l2*x0*y2 - l2*x2*y0 - l2*x1*y2 - l1*y0*y1 + l1*y1*y2)/(l2*x0 - l2*x2 - l1*y0 + l1*y2), z1 - (l3*(x1 - (l2*x0*x1 - l2*x1*x2 - l1*x0*y1 + l1*x0*y2 - l1*x2*y0 + l1*x2*y1)/(l2*x0 - l2*x2 - l1*y0 + l1*y2)))/l1];
end