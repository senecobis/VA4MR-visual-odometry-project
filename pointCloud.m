function P = pointCloud(img1, img2, p1, p2, K, T)

% This function takes as imputs the points correspondeces and gives the
% coordinates of the 3-D points; furtermore it draws a 3-D point cloud with
% the points correspondeces

% Made by Pippo809 as part of the programming assignement for Vision
% Algoritms for Mobile Robotics course, autumn 2021. ETH Zurich

%inputs p1 --> pixel coordinates of 1st image
%       p2 --> pixel coordinates of 2nd image
%       K  --> Intrinsic matrix
%       T  --> Transf. matrix from camera 1 to camera 2

%outputs: 
%       P --> [4xN] matrix containing the triangulated point cloud (in
% homogeneous coordinates), given by the function linearTriangulation

R = T(1:3,1:3);
t = T(:,4);

P = linearTriangulation(p1, p2,K*[eye(3),zeros(3,1)],K*T);

%% Visualize the 3-D scene
figure(1),
subplot(1,3,1)

% R,t should encode the pose of camera 2, such that M1 = [I|0] and M2=[R|t]


plot3(P(1,:), P(2,:), P(3,:), 'o');

% Display camera pose

plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');

center_cam2_W = -R'*t;
plotCoordinateFrame(R',center_cam2_W, 0.8);
text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');

axis equal
rotate3d on;
grid

% Display matched points
subplot(1,3,2)
imshow(img1,[]);
hold on
plot(p1(:,1), p1(:,2), 'ys');
title('Image 1')

subplot(1,3,3)
imshow(img2,[]);
hold on
plot(p2(:,1), p2(:,2), 'ys');
title('Image 2')

