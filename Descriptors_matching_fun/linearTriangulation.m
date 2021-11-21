function P = linearTriangulation(p1,p2,M1,M2)
% LINEARTRIANGULATION  Linear Triangulation
%
% Input:
%  - p1(N,2): coordinates of points in image 1
%  - p2(N,2): coordinates of points in image 2
%  - M1(3,4): projection matrix corresponding to first image
%  - M2(3,4): projection matrix corresponding to second image
%
% Output:
%  - P(4,N): homogeneous coordinates of 3-D points

P = zeros(4,size(p1,2));

p1(:,3) = ones(1,size(p1,1));
p1 = p1';

p2(:,3) = ones(1,size(p2,1));
p2 = p2';

for i = 1:size(p1,2)
    p1_mat = cross2Matrix(p1(:,i));
    p2_mat = cross2Matrix(p2(:,i));
    A = [p1_mat*M1;
         p2_mat*M2];
    [~,~,V] = svd(A);
    P(:,i) = V(:,end);    
end

P(1:4,:) = P(1:4,:)./P(4,:);  %rinormalizzazione

function M = cross2Matrix(x)
% CROSS2MATRIX  Antisymmetric matrix corresponding to a 3-vector
%
% Computes the antisymmetric matrix M corresponding to a 3-vector x such
% that M*y = cross(x,y) for all 3-vectors y.
%
% Input: 
%   - x(3,1) : vector
%
% Output: 
%   - M(3,3) : antisymmetric matrix
%

M = [0    -x(3)  x(2);
     x(3)   0   -x(1);
    -x(2)  x(1)   0  ];

