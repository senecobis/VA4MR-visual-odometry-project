function [R,T] = disambiguateRelativePose(Rots,u3,points0_h,points1_h,K1,K2)
% DISAMBIGUATERELATIVEPOSE- finds the correct relative camera pose (among
% four possible configurations) by returning the one that yields points
% lying in front of the image plane (with positive depth).
%
% Arguments:
%   Rots -  3x3x2: the two possible rotations returned by decomposeEssentialMatrix
%   u3   -  a 3x1 vector with the translation information returned by decomposeEssentialMatrix
%   p1   -  3xN homogeneous coordinates of point correspondences in image 1
%   p2   -  3xN homogeneous coordinates of point correspondences in image 2
%   K1   -  3x3 calibration matrix for camera 1
%   K2   -  3x3 calibration matrix for camera 2
%
% Returns:
%   R -  3x3 the correct rotation matrix
%   T -  3x1 the correct translation vector
%
%   where [R|t] = T_C2_C1 = T_C2_W is a transformation that maps points
%   from the world coordinate system (identical to the coordinate system of camera 1)
%   to camera 2.
%

index = zeros(2);
for point = 1:size(points0_h,2)
    
    for i = 1:2
        for j = 1:2
            
            M1 = K1*[eye(3), zeros(3,1)];
            M2 = K2*[Rots(:,:,i), u3*(-1)^j];
            
            P = linearTriangulation(points0_h(:,point),points1_h(:,point),M1,M2);
            
            if P > 0
                index(i,j) = index(i,j) + 1;
            end
        end
    end
end
[~,I] = max(index,[],[1 2],'linear');
[row,col] = ind2sub(2,I);

R = Rots(:,:,col);
T = u3*(-1)^row;
    


function P = linearTriangulation(p1,p2,M1,M2)
% LINEARTRIANGULATION  Linear Triangulation
%
% Input:
%  - p1(3,N): homogeneous coordinates of points in image 1
%  - p2(3,N): homogeneous coordinates of points in image 2
%  - M1(3,4): projection matrix corresponding to first image
%  - M2(3,4): projection matrix corresponding to second image
%
% Output:
%  - P(4,N): homogeneous coordinates of 3-D points

P = zeros(4,size(p1,2));

for i = 1:size(p1,2)
    p1_mat = cross2Matrix(p1(:,i));
    p2_mat = cross2Matrix(p2(:,i));
    A = [p1_mat*M1;
         p2_mat*M2];
    [~,~,V] = svd(A);
    P(:,i) = V(:,end);    
end

P(1:4,:) = P(1:4,:)./P(4,:);  %rinormalizzazione



