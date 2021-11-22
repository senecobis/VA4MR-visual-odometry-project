function [S, T_w_c] = processFrame(S0, img0, img1, K)
% The continuous VO pipeline is the core component of the proposed VO implementation. 
% Its responsibilities are three-fold:
% 1. Associate keypoints in the current frame to previously triangulated landmarks.
% 2. Based on this, estimate the current camera pose.
% 3. Regularly triangulate new landmarks using keypoints not associated to previously triangulated
% landmarks.
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function to process each frame, handler of the continuos VO pipeline
% input --> the 2 images RGB or GRAYSCALE, and the previous state
% output --> S : the state (a struct) containing a [2,k] matrix rapresenting 2d keypoint coordinates and a 
% [3,k] matrix rapresenting 3d landmarks coordinates. 
% T_w_c : the transformation from the world to cameras frame of img1 with respect to img0

% Made by senecobis :rpellerito@ethz.ch as part of the programming assignement
% for Vision Algoritms for Mobile Robotics course, autumn 2021. ETH Zurich
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [p1_1, p1_2, ..., p1_k] % association matrices
% [p2_4, p2_9, ..., p2_k-5] % p1_1 can be associated with an arbitrary p2_i
% [x1,   x2,   ..., xk]     % but each landmark is associated (index-wise) to the relative p1_i 
% we will use a struct 'S' with fields p and X
k = width(S0.p); % number of matches
S.p = zeros(2,k); % 2d coordinates
S.X = zeros(3,k); % 3d landmarks coordinates
[p1,p2] = matchKeypoints(im2gray(img0),im2gray(img1)); % p1 is matched with p2

[R,t, inliers] = findInitialPose(p1, p2, K);
T_w_c = [R,t];
keypoints_img1 = p1(inliers,:)';
keypoints_img2 = p2(inliers,:)';

%% sistemare qua -Lollo

idx = ismembertol(keypoints_img1', S0.p',0.01,'ByRows',1)'; % find the keypoint of the old image in the old state
% return the index of the specific keypoint i in state 0
S.p = keypoints_img2(:,idx); % the new keypoint will be the one associated with p1_i
%S.X = S0.X(:,idx);

p1_ho = [keypoints_img1; ones(1,size(keypoints_img1,2))];
p2_ho = [keypoints_img2; ones(1,size(keypoints_img2,2))];
S.X = linearTriangulation(p1_ho , p2_ho , K*eye(3,4), K*T_w_c);






end