function camera_matrix = estimateCameraProjectionMatrix(imagePoints,worldPoints)
% 功能： 已知非共面的3D-2D对应点估计相机投影矩阵，即投影透视矩阵，大小为3*4矩阵
% 输入：imagePoints, n*2 double ，图像坐标,[x,y]
%       worldPoints, n*3 double, 世界坐标,[x,y，z],其中z 不全为 0
% 输出： 
%       camera_matrix，3*4 double 投影矩阵，K[R,T]形式,[m11,m12,m13,m14;m21,m22,m23,m24;m31,m32,m33,m34];
%
% 注意：不考虑畸变影响，点顺序要对应,假设m34 = 1,则共11个未知数
% 内参K的计算调用的接口，测试与opencv接口一样的结果
%
% reference: https://mycourses.aalto.fi/pluginfile.php/578255/mod_resource/content/3/lecture7.pdf
% https://www.uio.no/studier/emner/matnat/its/UNIK4690/v16/forelesninger/lecture_5_1-the-camera-matrix-p.pdf
% https://blog.csdn.net/yangdashi888/article/details/51356385
% author:cuixingxing
% email: cuixingxing150@gmail.com
% 2018.8.1 实现
%
% 2022.9 注意：此函数功能已在Matlab R2022b及后续版本集成为内建函数`estimateCameraProjection`，
%           两者都输出为camera_matrix为3*4大小矩阵，等价相同（除去数值计算误差影响）！
%

n = size(imagePoints,1);
assert( n>=6 && size(worldPoints,1)==n );

%% 图像和世界坐标点归一化
% 1,图像坐标归一化
x = imagePoints(:,1);y = imagePoints(:,2);
x_avg = mean(x);y_avg = mean(y);
d_avg = mean(sqrt((x-x_avg).^2+(y-y_avg).^2));
T = [sqrt(2)/d_avg,0,-sqrt(2)*x_avg/d_avg;
    0,sqrt(2)/d_avg,-sqrt(2)*y_avg/d_avg;
    0,0,1];
imagePoints_norm = T*[imagePoints';ones(1,n)];
imagePoints_norm = imagePoints_norm(1:2,:)';
% 2，世界坐标归一化
X = worldPoints(:,1);Y = worldPoints(:,2);Z = worldPoints(:,3);
X_avg = mean(X);Y_avg = mean(Y);Z_avg = mean(Z);
D_avg = mean( sqrt( (X-X_avg).^2+(Y-Y_avg).^2 +(Z-Z_avg).^2) );
U = [sqrt(3)/D_avg,0,0,-sqrt(3)*X_avg/D_avg;
    0,sqrt(3)/D_avg,0,-sqrt(3)*Y_avg/D_avg;
    0,0,sqrt(3)/D_avg,-sqrt(3)*Z_avg/D_avg;
    0,0,0,1];
worldPoints_norm = U*[worldPoints';ones(1,n)];
worldPoints_norm = worldPoints_norm(1:3,:)';

%% DLT 算法
% m34 = 1;
A = zeros(2*n,12);
for i = 1:n
    X = worldPoints_norm(i,1);Y = worldPoints_norm(i,2);Z = worldPoints_norm(i,3);
    u = imagePoints_norm(i,1);v = imagePoints_norm(i,2);
    A(2*i-1,:) = [X,Y,Z,1,0,0,0,0,-u*X,-u*Y,-u*Z,-u];
    A(2*i,:)  = [0,0,0,0,X,Y,Z,1,-v*X,-v*Y,-v*Z,-v];
end
[~,~,V] = svd(A);
M = V(:,end);% M = [m11;m12;m13;m14;m21;m22;m23;m24;m31;m32;m33;m34]
camera_matrix_norm = reshape(M,4,3)';
camera_matrix = T\(camera_matrix_norm*U); % 反归一化, M = inv(T)*M_norm*U
end