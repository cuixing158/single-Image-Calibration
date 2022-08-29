function [K,extrinsicR,extrinsicT]= Pdecomp(P)
% Brief: 相机投影矩阵P分解为内外参
% Details:
%    None
% 
% Syntax:  
%     [K,extrinsicR,extrinsicT]= Pdecomp(P)
% 
% Inputs:
%    P - [3,4] size,[double] type,camera projection matrix
% 
% Outputs:
%    K - [3,3] size,[double] type,[fx,s,u0;0,fy,v0;0,0,1]
%    extrinsicR - [3,3] size,[double] type,rotation matrix
%    extrinsicT - [3,1] size,[double] type,translation vector
% 
% Example: 
%    None
% 
% See also: None
% References:
% [1] https://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/EPSRC_SSAZ/node5.html
% [2] https://ksimek.github.io/2012/08/14/decompose/

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         27-Aug-2022 16:14:24
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
%

%  R, IminusC:  3x3 orthogonal matrix R and 3x4 matrix IminusC
%               such that P=K*R*IminusC where C is the focal spot and
%               IminusC=[eye(3), -C]
M=P(:,1:3,1);
L=[.5,.5,0]*sqrt(sum(M.^2,2));
Dm=diag([1,1,L]); %preconditioner
[Km,R]=rqdecomp(Dm*M);
E=Km\(Dm*P);%  E: extrincsic matrix 3x4, [R,T] format
IminusC=R\E;
K=Km;%  K: intrinsic matrix 3x3
K(end,:)=K(end,:)/L;

K = K./K(end);
extrinsicR = R;
extrinsicT = R*IminusC(:,end);


function [R,Q]=rqdecomp(A)
%RQ decomposition of matrix A
%
%    [R,Q]=rqdecomp(A)

[Q,R]=qr(fliplr(A.'),0);

Q=fliplr(Q).';
R=rot90(R,2).';

c=sign(diag(R));
c(~c)=1;
C=spardiag(c);

R=R*C;
Q=C*Q;


function M=spardiag(V)
%makes a sparse n-by-n diagonal matrix with V on the diagonal where V
%is length n.
%
% M=spardiag(V)

if ~isa(V,'double') || ~islogical(V)
    V=double(V);
end

N=numel(V);

M=spdiags(V(:),0,N,N);