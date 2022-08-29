function K = calIntrinsicFrom3VanishingPts(vanishingPt1,vanishingPt2,vanishingPt3)
% Brief: 从三个消失点坐标求解相机内参K(代数求解方法)
% Details:
%    代数求解方法，已知三个消失点坐标(u1,u2,u3)',(v1,v2,v3)',(w1,w2,w3)',求取
% 内参K中的f,u0,v0. 根据正交约束，满足方程u'*(inv(K))'*inv(K)*v=0,三个消失点有
% 三对组合，对应三个独立方程，可求解焦距f，相机主点u0,v0. 
% 理论详见"相机标定及立体视觉文献"文件夹下的《2020_(相机标定_重要_重建较好)_lec14_calibration.pdf》
% 第50张幻灯片叙述.
% 条件：由三个消失点组成的三角形必定为锐角三角形
%
% syms f u0 v0 v1 v2 u1 u2 w1 w2 real
% assume(f>0)
% K = [f,0,u0;0,f,v0;0,0,1];
% v = [v1,v2,1]';
% u = [u1,u2,1]';
% w = [w1,w2,1]';
% 
% express1 = simplify(v'*(inv(K))'*inv(K)*u);
% express2 = subs(express1,u,w);
% express3 = subs(express1,v,w);
% [f_,u0_,v0_,params,conditions] = solve([express1,express2,express3],[f,u0,v0],...
%     ReturnConditions=true,...
%     IgnoreAnalyticConstraints=true)
% 
% Syntax:  
%     K = calIntrinsicFrom3VanishingPts(vanishingPt1,vanshingPt2,vanshingPt3)
% 
% Inputs:
%    vanishingPt1 - [1,3] size,[double] type,最后一列为1，齐次性
%    vanishingPt2 - [1,3] size,[double] type,最后一列为1，齐次性
%    vanishingPt3 - [1,3] size,[double] type,最后一列为1，齐次性
% 
% Outputs:
%    K - [3,3] size,[double] type,通用格式,[f,0,u0;0,f,v0;0,0,1]
% 
% Example: 
%    None
% 
% See also: None

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         22-Aug-2022 09:44:23
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
%
arguments
    vanishingPt1 (1,3) double
    vanishingPt2 (1,3) double
    vanishingPt3 (1,3) double
end
u1 = vanishingPt1(1,1);u2 = vanishingPt1(1,2);u3 = vanishingPt1(1,3);
v1 = vanishingPt2(1,1);v2 = vanishingPt2(1,2);v3 = vanishingPt2(1,3);
w1 = vanishingPt3(1,1);w2 = vanishingPt3(1,2);w3 = vanishingPt3(1,3);

% 直接获取结果
f = (abs((u1*v2*w3 - u1*v3*w2 - u2*v1*w3 + u2*v3*w1 + u3*v1*w2 - u3*v2*w1))*abs(u3)*abs(v3)*abs(w3)*sqrt(- u1^4*v1^2*v3^2*w3^4 + 2*u1^4*v1*v3^3*w1*w3^3 - u1^4*v3^4*w1^2*w3^2 - 2*u1^3*u2*v1*v2*v3^2*w3^4 + 2*u1^3*u2*v1*v3^3*w2*w3^3 + 2*u1^3*u2*v2*v3^3*w1*w3^3 - 2*u1^3*u2*v3^4*w1*w2*w3^2 + 2*u1^3*u3*v1^3*v3*w3^4 - 2*u1^3*u3*v1^2*v3^2*w1*w3^3 + u1^3*u3*v1*v2^2*v3*w3^4 - 2*u1^3*u3*v1*v3^3*w1^2*w3^2 - u1^3*u3*v1*v3^3*w2^2*w3^2 - u1^3*u3*v2^2*v3^2*w1*w3^3 + 2*u1^3*u3*v3^4*w1^3*w3 + u1^3*u3*v3^4*w1*w2^2*w3 - u1^2*u2^2*v1^2*v3^2*w3^4 + 2*u1^2*u2^2*v1*v3^3*w1*w3^3 - u1^2*u2^2*v2^2*v3^2*w3^4 + 2*u1^2*u2^2*v2*v3^3*w2*w3^3 - u1^2*u2^2*v3^4*w1^2*w3^2 - u1^2*u2^2*v3^4*w2^2*w3^2 + 4*u1^2*u2*u3*v1^2*v2*v3*w3^4 - 2*u1^2*u2*u3*v1^2*v3^2*w2*w3^3 - 2*u1^2*u2*u3*v1*v2*v3^2*w1*w3^3 - 2*u1^2*u2*u3*v1*v3^3*w1*w2*w3^2 + u1^2*u2*u3*v2^3*v3*w3^4 - u1^2*u2*u3*v2^2*v3^2*w2*w3^3 - 2*u1^2*u2*u3*v2*v3^3*w1^2*w3^2 - u1^2*u2*u3*v2*v3^3*w2^2*w3^2 + 4*u1^2*u2*u3*v3^4*w1^2*w2*w3 + u1^2*u2*u3*v3^4*w2^3*w3 - u1^2*u3^2*v1^4*w3^4 - 2*u1^2*u3^2*v1^3*v3*w1*w3^3 - u1^2*u3^2*v1^2*v2^2*w3^4 - 2*u1^2*u3^2*v1^2*v2*v3*w2*w3^3 + 6*u1^2*u3^2*v1^2*v3^2*w1^2*w3^2 + 2*u1^2*u3^2*v1^2*v3^2*w2^2*w3^2 - u1^2*u3^2*v1*v2^2*v3*w1*w3^3 + 4*u1^2*u3^2*v1*v2*v3^2*w1*w2*w3^2 - 2*u1^2*u3^2*v1*v3^3*w1^3*w3 - u1^2*u3^2*v1*v3^3*w1*w2^2*w3 - u1^2*u3^2*v2^3*v3*w2*w3^3 + 2*u1^2*u3^2*v2^2*v3^2*w1^2*w3^2 + 2*u1^2*u3^2*v2^2*v3^2*w2^2*w3^2 - 2*u1^2*u3^2*v2*v3^3*w1^2*w2*w3 - u1^2*u3^2*v2*v3^3*w2^3*w3 - u1^2*u3^2*v3^4*w1^4 - u1^2*u3^2*v3^4*w1^2*w2^2 - 2*u1*u2^3*v1*v2*v3^2*w3^4 + 2*u1*u2^3*v1*v3^3*w2*w3^3 + 2*u1*u2^3*v2*v3^3*w1*w3^3 - 2*u1*u2^3*v3^4*w1*w2*w3^2 + u1*u2^2*u3*v1^3*v3*w3^4 - u1*u2^2*u3*v1^2*v3^2*w1*w3^3 + 4*u1*u2^2*u3*v1*v2^2*v3*w3^4 - 2*u1*u2^2*u3*v1*v2*v3^2*w2*w3^3 - u1*u2^2*u3*v1*v3^3*w1^2*w3^2 - 2*u1*u2^2*u3*v1*v3^3*w2^2*w3^2 - 2*u1*u2^2*u3*v2^2*v3^2*w1*w3^3 - 2*u1*u2^2*u3*v2*v3^3*w1*w2*w3^2 + u1*u2^2*u3*v3^4*w1^3*w3 + 4*u1*u2^2*u3*v3^4*w1*w2^2*w3 - 2*u1*u2*u3^2*v1^3*v2*w3^4 - 2*u1*u2*u3^2*v1^2*v2*v3*w1*w3^3 + 4*u1*u2*u3^2*v1^2*v3^2*w1*w2*w3^2 - 2*u1*u2*u3^2*v1*v2^3*w3^4 - 2*u1*u2*u3^2*v1*v2^2*v3*w2*w3^3 + 4*u1*u2*u3^2*v1*v2*v3^2*w1^2*w3^2 + 4*u1*u2*u3^2*v1*v2*v3^2*w2^2*w3^2 - 2*u1*u2*u3^2*v1*v3^3*w1^2*w2*w3 + 4*u1*u2*u3^2*v2^2*v3^2*w1*w2*w3^2 - 2*u1*u2*u3^2*v2*v3^3*w1*w2^2*w3 - 2*u1*u2*u3^2*v3^4*w1^3*w2 - 2*u1*u2*u3^2*v3^4*w1*w2^3 + 2*u1*u3^3*v1^4*w1*w3^3 + 2*u1*u3^3*v1^3*v2*w2*w3^3 - 2*u1*u3^3*v1^3*v3*w1^2*w3^2 - u1*u3^3*v1^3*v3*w2^2*w3^2 + 2*u1*u3^3*v1^2*v2^2*w1*w3^3 - 2*u1*u3^3*v1^2*v2*v3*w1*w2*w3^2 - 2*u1*u3^3*v1^2*v3^2*w1^3*w3 - u1*u3^3*v1^2*v3^2*w1*w2^2*w3 + 2*u1*u3^3*v1*v2^3*w2*w3^3 - u1*u3^3*v1*v2^2*v3*w1^2*w3^2 - 2*u1*u3^3*v1*v2^2*v3*w2^2*w3^2 - 2*u1*u3^3*v1*v2*v3^2*w1^2*w2*w3 + 2*u1*u3^3*v1*v3^3*w1^4 + 2*u1*u3^3*v1*v3^3*w1^2*w2^2 - u1*u3^3*v2^2*v3^2*w1^3*w3 - 2*u1*u3^3*v2^2*v3^2*w1*w2^2*w3 + 2*u1*u3^3*v2*v3^3*w1^3*w2 + 2*u1*u3^3*v2*v3^3*w1*w2^3 - u2^4*v2^2*v3^2*w3^4 + 2*u2^4*v2*v3^3*w2*w3^3 - u2^4*v3^4*w2^2*w3^2 + u2^3*u3*v1^2*v2*v3*w3^4 - u2^3*u3*v1^2*v3^2*w2*w3^3 + 2*u2^3*u3*v2^3*v3*w3^4 - 2*u2^3*u3*v2^2*v3^2*w2*w3^3 - u2^3*u3*v2*v3^3*w1^2*w3^2 - 2*u2^3*u3*v2*v3^3*w2^2*w3^2 + u2^3*u3*v3^4*w1^2*w2*w3 + 2*u2^3*u3*v3^4*w2^3*w3 - u2^2*u3^2*v1^3*v3*w1*w3^3 - u2^2*u3^2*v1^2*v2^2*w3^4 - u2^2*u3^2*v1^2*v2*v3*w2*w3^3 + 2*u2^2*u3^2*v1^2*v3^2*w1^2*w3^2 + 2*u2^2*u3^2*v1^2*v3^2*w2^2*w3^2 - 2*u2^2*u3^2*v1*v2^2*v3*w1*w3^3 + 4*u2^2*u3^2*v1*v2*v3^2*w1*w2*w3^2 - u2^2*u3^2*v1*v3^3*w1^3*w3 - 2*u2^2*u3^2*v1*v3^3*w1*w2^2*w3 - u2^2*u3^2*v2^4*w3^4 - 2*u2^2*u3^2*v2^3*v3*w2*w3^3 + 2*u2^2*u3^2*v2^2*v3^2*w1^2*w3^2 + 6*u2^2*u3^2*v2^2*v3^2*w2^2*w3^2 - u2^2*u3^2*v2*v3^3*w1^2*w2*w3 - 2*u2^2*u3^2*v2*v3^3*w2^3*w3 - u2^2*u3^2*v3^4*w1^2*w2^2 - u2^2*u3^2*v3^4*w2^4 + 2*u2*u3^3*v1^3*v2*w1*w3^3 + 2*u2*u3^3*v1^2*v2^2*w2*w3^3 - 2*u2*u3^3*v1^2*v2*v3*w1^2*w3^2 - u2*u3^3*v1^2*v2*v3*w2^2*w3^2 - 2*u2*u3^3*v1^2*v3^2*w1^2*w2*w3 - u2*u3^3*v1^2*v3^2*w2^3*w3 + 2*u2*u3^3*v1*v2^3*w1*w3^3 - 2*u2*u3^3*v1*v2^2*v3*w1*w2*w3^2 - 2*u2*u3^3*v1*v2*v3^2*w1*w2^2*w3 + 2*u2*u3^3*v1*v3^3*w1^3*w2 + 2*u2*u3^3*v1*v3^3*w1*w2^3 + 2*u2*u3^3*v2^4*w2*w3^3 - u2*u3^3*v2^3*v3*w1^2*w3^2 - 2*u2*u3^3*v2^3*v3*w2^2*w3^2 - u2*u3^3*v2^2*v3^2*w1^2*w2*w3 - 2*u2*u3^3*v2^2*v3^2*w2^3*w3 + 2*u2*u3^3*v2*v3^3*w1^2*w2^2 + 2*u2*u3^3*v2*v3^3*w2^4 - u3^4*v1^4*w1^2*w3^2 - 2*u3^4*v1^3*v2*w1*w2*w3^2 + 2*u3^4*v1^3*v3*w1^3*w3 + u3^4*v1^3*v3*w1*w2^2*w3 - u3^4*v1^2*v2^2*w1^2*w3^2 - u3^4*v1^2*v2^2*w2^2*w3^2 + 4*u3^4*v1^2*v2*v3*w1^2*w2*w3 + u3^4*v1^2*v2*v3*w2^3*w3 - u3^4*v1^2*v3^2*w1^4 - u3^4*v1^2*v3^2*w1^2*w2^2 - 2*u3^4*v1*v2^3*w1*w2*w3^2 + u3^4*v1*v2^2*v3*w1^3*w3 + 4*u3^4*v1*v2^2*v3*w1*w2^2*w3 - 2*u3^4*v1*v2*v3^2*w1^3*w2 - 2*u3^4*v1*v2*v3^2*w1*w2^3 - u3^4*v2^4*w2^2*w3^2 + u3^4*v2^3*v3*w1^2*w2*w3 + 2*u3^4*v2^3*v3*w2^3*w3 - u3^4*v2^2*v3^2*w1^2*w2^2 - u3^4*v2^2*v3^2*w2^4))/(u3^2*v3^2*w3^2*(u1*v2*w3 - u1*v3*w2 - u2*v1*w3 + u2*v3*w1 + u3*v1*w2 - u3*v2*w1)^2);
u0 = (- u2^2*v2*v3*w3^2 + u2^2*v3^2*w2*w3 + u2*u3*v2^2*w3^2 - u2*u3*v3^2*w2^2 + u1*w1*u2*v3^2*w3 - u1*v1*u2*v3*w3^2 - u3^2*v2^2*w2*w3 + u3^2*v2*v3*w2^2 - v1*w1*u3^2*v2*w3 + v1*w1*u3^2*v3*w2 + u1*v1*u3*v2*w3^2 - u1*w1*u3*v3^2*w2)/(u3*v3*w3*(u1*v2*w3 - u1*v3*w2 - u2*v1*w3 + u2*v3*w1 + u3*v1*w2 - u3*v2*w1));
v0 = -(- u1^2*v1*v3*w3^2 + u1^2*v3^2*w1*w3 + u1*u3*v1^2*w3^2 - u1*u3*v3^2*w1^2 + u2*w2*u1*v3^2*w3 - u2*v2*u1*v3*w3^2 - u3^2*v1^2*w1*w3 + u3^2*v1*v3*w1^2 - v2*w2*u3^2*v1*w3 + v2*w2*u3^2*v3*w1 + u2*v2*u3*v1*w3^2 - u2*w2*u3*v3^2*w1)/(u3*v3*w3*(u1*v2*w3 - u1*v3*w2 - u2*v1*w3 + u2*v3*w1 + u3*v1*w2 - u3*v2*w1));

K = [f,0,u0;
    0,f,v0;
    0,0,1];
end