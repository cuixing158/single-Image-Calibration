function [camera_matrix,K, extrinsicRotMatrix, extrinsicT] = oneImageCalibrate3D(imagePoints,worldPoints)
% 功能： 根据对应的非共面3D-2D点集，获取相机内外参和投影矩阵，即投影透视矩阵，大小为3*4
% 输入：imagePoints, n*2 double ，图像坐标,[x,y]
%       worldPoints, n*3 double, 世界坐标,[x,y，z],其中z 不全为 0
% 输出： K，相机内参矩阵，注意必须为[fx,s,cx;0,fy,cy;0,0,1]形式
%       camera_matrix，3*4 double 投影矩阵，K[R,T]形式,[m11,m12,m13,m14;m21,m22,m23,m24;m31,m32,m33,m34];
%       extrinsicRotMatrix ,3*3 double 外参旋转矩阵
%       extrinsicT ，3*1 double 外参平移向量

%
% author:cuixingxing
% email: cuixingxing150@gmail.com
% 2022.8.28
% 针孔相机线性标定方法

% or use matlab build-in function estimateCameraMatrix, but note:The outputs are transposed to each other
camera_matrix = estimateCameraProjectionMatrix(imagePoints,worldPoints);

% P = K[R,T]
[K,extrinsicRotMatrix,extrinsicT]= Pdecomp(camera_matrix);

