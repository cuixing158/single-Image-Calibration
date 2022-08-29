function [intrinsicK,extrinsicRotation,extrinsicTranslation] = tsaiCalibrate(...
    imagePoints,worldPoints,principlePoint)
% Brief: tasi's method camera calibration from corresponding image and world
% coordinate points.
% Details:
%    The linear calibration part of the tasi method.Returnd params is
%    general form, not computer vision toolbox transposed form. Tsai's
%    calibration method.
%    http://people.csail.mit.edu/bkph/articles/Tsai_Revisited.pdf 
% 
% Syntax:  
%     [intrinsicK,extrinsicRotation,extrinsicTranslation] =
%     tsaiCalibrate(imagePoints,worldPoints,principlePoint) 
% 
% Inputs:
%    imagePoints - [m,2] size,[double] type,2D pixel coordinates
%    worldPoints - [m,3] size,[double] type,3D world coordinates
%    principlePoint - [1,2] size,[double] type,[u0,v0],estimated pixel coordinalte.
% 
% Outputs:
%    intrinsicK - [3,3] size,[double] type,contains focal,principle point.
%    extrinsicRotation - [3,3] size,[double] type,orthogonal matrix.
%    extrinsicTranslation - [3,1] size,[double] type,translation vector.
% 
% Example: 
%    None 
% 
% See also: None
% 

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         12-Aug-2022 11:33:22
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
    imagePoints (:,2) double
    worldPoints (:,3) double
    principlePoint (1,2) double
end
rP = imagePoints(:,1)-principlePoint(1);  %x
cP = imagePoints(:,2)-principlePoint(2);  %y
xW = worldPoints(:,1);
yW = worldPoints(:,2);
zW = worldPoints(:,3);

%% Solve for the missing parameters
% pg 384 -> A matrix
A = [rP.*xW, rP.*yW, rP.*zW, rP, -cP.*xW, -cP.*yW, -cP.*zW, -cP];
%A = A(10:20,:);  % worse results when you use a subset

% pg 384 -> solve for xbar, using SVD
% xbar = [r21 r22 r23 Ty alpha*r11 alpha*r12 alpha*r13 alpha*Tx]
[U,S,V] = svd(A);   %#ok<ASGLU>
xbar = V(:,end); %grab the last column of V to get the solution corresponding to the smallest singular value

% pg 384 -> solve for abs(k) and alpha
kAbs = sum(xbar(1:3).^2).^0.5;
alpha = (sum(xbar(5:7).^2).^0.5)/kAbs;

% pg 384 -> find sign for k
% Choose k such that r/(r11*x+r12*y+r13*z+Tx >0)  NOTE: THIS IS DIFFERENT
% FROM BOOK.  SHOULD BE DIVIDE, and GREATER THAN
if sum(rP./( xbar(5).*xW +  xbar(6).*yW +  xbar(7).*zW +xbar(8)) ) > 0 
    k = -kAbs;
else
    k = kAbs;
end
k = - k;

% pg 384-385 -> Calculate R_est, col 3 calculated by cross product of
% col 1 and col 2
r1all = xbar(5:7)/(alpha*k);
r2all = xbar(1:3)/k;
r3all = cross(r1all,r2all);
R_est = [r1all,r2all,r3all]';
% RENORMALIZE
[Ur,Sr,Vr] = svd(R_est);
R_est = Ur*eye(size(Sr,1))*Vr';

% pg 384 -> use k and alpha to find Ty and Tx  
Ty = xbar(4)/k;  
Tx = xbar(8)/(alpha*k); 

% pg 385 -> find Tz and Fx
C = [rP, (R_est(1,1)*xW + R_est(1,2)*yW + R_est(1,3)*zW + Tx)];
d = -rP.*(R_est(3,1)*xW + R_est(3,2)*yW + R_est(3,3)*zW );

ybar = C\d;  %Solve C.ybar=d, ybar = [Tz,fx]

%maxErr = max(abs(C*ybar- d));
%display(maxErr)

Tz = ybar(1);
fxe = ybar(2);

% pg 385 -> calculate fy estimate, using alpha = f_x / f_y
fye = fxe/alpha; 

% pg 381 -> calculate T_est, using T = -R_w^c * O_c^w  from 11.2.1
% T_est =-R_est'*[Tx;Ty;Tz]; 

% output
intrinsicK = [fxe,0,principlePoint(1);
    0,fye,principlePoint(2);
    0,0,1];
extrinsicRotation = R_est;
extrinsicTranslation = [Tx;Ty;Tz];
end