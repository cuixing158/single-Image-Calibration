function extrinsicTranslation = calExtrinsicT(K,R,imagePts,worldPts)
% Brief: 由相机内参和外参旋转矩阵R，对应的3D-2D点集合，求解外参平移向量T
% Details:
%    数学原理分析见calExtrinsicAnalysis.mlx代数求解部分
% 
% Syntax:  
%     extrinsicTranslation = calExtrinsicT(K,R,imagePts,worldPts)
% 
% Inputs:
%    K - [3,3] size,[double] type,intrinsic matrix K
%    R - [3,3] size,[double] type,extrinsic rotation matrix
%    imagePts - [m,2] size,[double] type,image points
%    worldPts - [m,3] size,[double] type,corresponding world points
% 
% Outputs:
%    extrinsicTranslation - [3,1] size,[double] type,extrinsic tranlation
%    vector
% 
% Example: 
%    None
% 
% See also: None
% References:
% https://blog.csdn.net/MyArrow/article/details/53780972
% https://zhuanlan.zhihu.com/p/64273563

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         27-Aug-2022 14:19:07
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
%
f = K(1,1);
u0 = K(1,3);
v0 = K(2,3);
R11 = R(1,1);R12 = R(1,2);R13 = R(1,3);
R21 = R(2,1);R22 = R(2,2);R23 = R(2,3);
R31 = R(3,1);R32 = R(3,2);R33 = R(3,3);

numPts = size(imagePts,1);
A = zeros(2*numPts,3);
b = zeros(2*numPts,1);
for i = 1:numPts
    x1 = imagePts(i,1); x2 = imagePts(i,2);
    X1 = worldPts(i,1); X2 = worldPts(i,2); X3 = worldPts(i,3);

    A(2*i-1,:) = [f,0,u0-x1]; 
    b(2*i-1) = x1*(R31*X1+R32*X2+R33*X3)-X1*(R11*f+R31*u0)-X2*(R12*f+R32*u0)-X3*(R13*f+R33*u0);
    
    A(2*i,:) = [0,f,v0-x2]; 
    b(2*i) = x2*(R31*X1+R32*X2+R33*X3)-X1*(R21*f+R31*v0)-X2*(R22*f+R32*v0)-X3*(R23*f+R33*v0);
end
[U,S,V] = svd(A);
b_ = U'*b;% 2*numPts rows by 1 column
y = b_(1:3)./[S(1,1);S(2,2);S(3,3)];
x = V*y; % or use x = A\b
extrinsicTranslation = x;
end