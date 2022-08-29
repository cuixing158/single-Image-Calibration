function plotCameraProjection3D(srcImage,l11,l12,l21,l22,l31,l32,vanishingPts, ...
    geometricK,extrinsicRotation,extrinsicT,worldPoints,cornerMInCamera,...
    cornerNInCamera,P,Q)
% Brief: 根据几何求解的相机内外参，绘制相机3D投影成像仿真图
% Details:
%    为避免尺度导致坐标系比例观察悬殊太大影响，绘制实际桌子的深度（即Z轴）做了适当缩放， 
%  不影响总体成像过程。
% 
% Syntax:  
%     plotCameraProjection3D(srcImage,l11,l12,l21,l22,l31,l32,vanishingPts,geometricK,extrinsicRotation,extrinsicT,worldPoints,cornerMInCamera,cornerNInCamera,P,Q)
% 
% Inputs:
%    srcImage - [m,n] size,[double] type,Description
%    l11 - [m,n] size,[double] type,Description
%    l12 - [m,n] size,[double] type,Description
%    l21 - [m,n] size,[double] type,Description
%    l22 - [m,n] size,[double] type,Description
%    l31 - [m,n] size,[double] type,Description
%    l32 - [m,n] size,[double] type,Description
%    vanishingPts - [m,n] size,[double] type,消失点
%    geometricK - [m,n] size,[double] type,相机内参
%    extrinsicRotation - [m,n] size,[double] type,外参R
%    extrinsicT - [m,n] size,[double] type,外参T
%    worldPoints - [m,n] size,[double] type,已知单张图像中所有的3D点
%    cornerMInCamera - [m,n] size,[double] type,在相机像素坐标系下一已知3D-2D点
%    cornerNInCamera - [m,n] size,[double] type,在相机像素坐标系下另一已知3D-2D点
%    P - [1,2] size,[double] type,光心坐标原点[0,0,0]
%    Q - [1,2] size,[double] type,中间计算辅助点,[x,y,z]
% 
% Outputs:
%    None
% 
% Example: 
%    None
% 
% See also: None

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         27-Aug-2022 10:56:04
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
%

altitude = geometricK(1,1);
u0 = geometricK(1,3);
v0 = geometricK(2,3);

% plot 3D view
figure(Name="3D view",units='normalized',outerposition=[0 0 1 1]);
box off;grid off;hold on;axis equal;

[X,Y] = meshgrid(1:size(srcImage,2),1:size(srcImage,1));
Ximg = X - u0;
Yimg = Y - v0;
Zimg = altitude*ones(size(Ximg));
warp(Ximg,Yimg,Zimg,srcImage);

line11Data = [l11.Position-[u0,v0],altitude*ones(2,1)]; plot3(line11Data(:,1),line11Data(:,2),line11Data(:,3),'r',LineWidth=2);
line12Data = [l12.Position-[u0,v0],altitude*ones(2,1)]; plot3(line12Data(:,1),line12Data(:,2),line12Data(:,3),'r',LineWidth=2);
line21Data = [l21.Position-[u0,v0],altitude*ones(2,1)]; plot3(line21Data(:,1),line21Data(:,2),line21Data(:,3),'g',LineWidth=2);
line22Data = [l22.Position-[u0,v0],altitude*ones(2,1)]; plot3(line22Data(:,1),line22Data(:,2),line22Data(:,3),'g',LineWidth=2);
line31Data = [l31.Position-[u0,v0],altitude*ones(2,1)]; plot3(line31Data(:,1),line31Data(:,2),line31Data(:,3),'b',LineWidth=2);
line32Data = [l32.Position-[u0,v0],altitude*ones(2,1)]; plot3(line32Data(:,1),line32Data(:,2),line32Data(:,3),'b',LineWidth=2);

vanishingPts = [vanishingPts-[u0,v0],altitude*ones(3,1)];
text(vanishingPts(:,1),vanishingPts(:,2),vanishingPts(:,3),["A","B","C"],...
    FontSize=15,FontWeight="bold",Color='#7E2F8E')

axisScalar = 1.2;
axis([axisScalar*min(vanishingPts(:,1)),axisScalar*max(vanishingPts(:,1)),...
    axisScalar*min(vanishingPts(:,2)),axisScalar*max(vanishingPts(:,2)),...
    -(axisScalar-1)*altitude,axisScalar*altitude])
hAxis = gca;
hAxis.XRuler.FirstCrossoverValue  = 0; % X crossover with Y axis
hAxis.XRuler.SecondCrossoverValue = 0; % X crossover with Z axis
hAxis.YRuler.FirstCrossoverValue  = 0; % Y crossover with X axis
hAxis.YRuler.SecondCrossoverValue = 0; % Y crossover with Z axis
hAxis.ZRuler.FirstCrossoverValue  = 0; % Z crossover with X axis
hAxis.ZRuler.SecondCrossoverValue = 0; % Z crossover with Y axis
hAxis.Position = [0,0,1,1];

vertices = [[0,0,0];vanishingPts];
tetramesh([1,2,3,4],vertices,FaceAlpha=0.15,LineWidth=2,EdgeColor='#7E2F8E')
text(vertices(1,1),vertices(1,2),vertices(1,3),'P',...
    FontWeight='bold',FontSize=15)

% plot base planar,ie the plane in which triangle ABC is located(optional visualization)
polyin = polyshape([vanishingPts(:,1:2)]);
[xlim,ylim] = boundingbox(polyin);
[XLIM,YLIM] = meshgrid(xlim,ylim);
index=[1,2,4,3];
x = XLIM(index);
y = YLIM(index);
z = altitude*ones(size(x));
color = [0.4940 0.1840 0.5560];
patch(x,y,z,color,FaceAlpha=0.10,DisplayName="base image plane")

% adjust to show axes label
xlabel("Xc");ylabel("Yc");zlabel("Zc")
hAxis.YDir ="reverse";
hAxis.ZDir = "reverse";
hAxis.XAxis.Label.Position=[hAxis.XLim(end),v0,0];
hAxis.YAxis.Label.Position=[u0,hAxis.YLim(end),0];
hAxis.ZAxis.Label.Position=[u0,v0,hAxis.ZLim(end)];
hAxis.LabelFontSizeMultiplier=2.5;
plotCamera(AbsolutePose=rigid3d(eye(3),[0,0,0]),Size=210,Color='red',...
    AxesVisible=1,Label="相机光心");

% plot "table" object in 3D camera world
scalarTable = 18;% 深度缩放因子，方便整体查看，理论值应该为altitude，即焦距f,像素单位
tableWorldPts = worldPoints(1:4,:);
tableConersInCamera = extrinsicRotation*tableWorldPts'+extrinsicT';
tableConersInCameraPixel = (scalarTable*tableConersInCamera)';

closedPts = [tableConersInCameraPixel;tableConersInCameraPixel(1,:)];
plot3(closedPts(:,1),closedPts(:,2),closedPts(:,3),LineWidth=2);% 实际桌子边在相机物理坐标系下的绘图
text(closedPts(1,1),closedPts(1,2),closedPts(1,3),'Mw',FontSize=15,FontWeight="bold")
text(closedPts(2,1),closedPts(2,2),closedPts(2,3),'Nw',FontSize=15,FontWeight="bold")
for i = 1:4 % 桌子四条边的角点投影线绘制
    projectLine = [P;tableConersInCameraPixel(i,:)];
    plot3(projectLine(:,1),projectLine(:,2),projectLine(:,3),'k--');
end
PMCoord = [P;cornerMInCamera];
PNCoord = [P;cornerNInCamera];
plot3(PMCoord(:,1),PMCoord(:,2),PMCoord(:,3),'k--');
plot3(PNCoord(:,1),PNCoord(:,2),PNCoord(:,3),'k--');
scatter3(Q(1,1),Q(1,2),Q(1,3),'filled',MarkerEdgeColor='k',MarkerFaceColor=[0 .75 .75]);
text(cornerMInCamera(:,1),cornerMInCamera(:,2),cornerMInCamera(:,3),'M',FontSize=15,FontWeight="bold")
text(cornerNInCamera(:,1),cornerNInCamera(:,2),cornerNInCamera(:,3),'N',FontSize=15,FontWeight="bold")
text(Q(:,1),Q(:,2),Q(:,3),'Q',FontSize=15,FontWeight="bold")
zoom(0.25);
hAxis.ZAxis.Label.Position=[u0,v0,hAxis.ZLim(end)];
hAxis.ZLim = [-2000,hAxis.ZLim(end)];
title("相机像素坐标系下成像投影")
end