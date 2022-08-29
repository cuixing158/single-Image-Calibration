function [camera_matrix,K, extrinsicRotMatrix, extrinsicT] = oneImageCalibrate3D(imagePoints,worldPoints)
% ���ܣ� ���ݶ�Ӧ�ķǹ���3D-2D�㼯����ȡ�������κ�ͶӰ���󣬼�ͶӰ͸�Ӿ��󣬴�СΪ3*4
% ���룺imagePoints, n*2 double ��ͼ������,[x,y]
%       worldPoints, n*3 double, ��������,[x,y��z],����z ��ȫΪ 0
% ����� K������ڲξ���ע�����Ϊ[fx,s,cx;0,fy,cy;0,0,1]��ʽ
%       camera_matrix��3*4 double ͶӰ����K[R,T]��ʽ,[m11,m12,m13,m14;m21,m22,m23,m24;m31,m32,m33,m34];
%       extrinsicRotMatrix ,3*3 double �����ת����
%       extrinsicT ��3*1 double ���ƽ������

%
% author:cuixingxing
% email: cuixingxing150@gmail.com
% 2022.8.28
% ���������Ա궨����

% or use matlab build-in function estimateCameraMatrix, but note:The outputs are transposed to each other
camera_matrix = estimateCameraProjectionMatrix(imagePoints,worldPoints);

% P = K[R,T]
[K,extrinsicRotMatrix,extrinsicT]= Pdecomp(camera_matrix);

