function [instrinsicK,extrinsicR,extrinsicT] = cameraProjectionMatDecomp(cameraMatrix)
% Brief: 利用P = K*[R,T]，内外参性质公式求解
% Details:
%    原理见2011年斯坦福李飞飞老师课件
% http://vision.stanford.edu/teaching/cs231a_autumn1112/lecture/lecture8_camera_calibration_cs231a_marked.pdf
% 
% Syntax:  
%     [instrinsicK,extrinsicR,extrinsicT] = cameraProjectionMatDecomp(cameraMatrix)
% 
% Inputs:
%    cameraMatrix - [3,4] size,[double] type,camera projection matrix
% 
% Outputs:
%    instrinsicK - [3,3] size,[double] type,[alpha,s,u0;0,beta,v0;0,0,1]
%    extrinsicR - [3,3] size,[double] type,rotation
%    extrinsicT - [3,1] size,[double] type,translation
% 
% Example: 
%    None
% 
% See also: None

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         28-Aug-2022 14:26:15
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
%

A = cameraMatrix(:,1:3);
b = cameraMatrix(:,4);

a1 = A(1,:)';
a2 = A(2,:)';
a3 = A(3,:)';

% principle point [u0,v0]
rho2 = 1./dot(a3,a3);
u0 = rho2*dot(a1,a3);
v0 = rho2*dot(a2,a3);

% calculate cos(\theta) and sin(\theta)
a13OuterPro = cross(a1,a3);
a23OuterPro = cross(a2,a3);
cosTheta = dot(a13OuterPro,a23OuterPro)./(vecnorm(a13OuterPro)*vecnorm(a23OuterPro));
sinTheta = sqrt(1-cosTheta.^2);
cotTheta = cosTheta./sinTheta;

% intrinsic
alpha = rho2*vecnorm(a13OuterPro)*sinTheta;
beta = rho2*vecnorm(a23OuterPro)*sinTheta;

% extrinsic
r1 = a23OuterPro./vecnorm(a23OuterPro);
r3 = a3./vecnorm(a3);
r2 = cross(r3,r1);

rho = sqrt(rho2);
K = [alpha,-alpha*cotTheta,u0;
    0,     beta./sinTheta, v0;
    0,     0,               1];
T = rho*(K\b);

% output intrinsic and extrinsic matrix
instrinsicK = K;
extrinsicR = [r1';
    r2';
    r3'];
extrinsicT = T;
end