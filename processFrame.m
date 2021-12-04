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
% for Vision Algoritms for Mobile Robotics course, fall 2021. ETH Zurich
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [p1_1, p1_2, ..., p1_k] % association matrices
% [p2_4, p2_9, ..., p2_k-5] % p1_1 can be associated with an arbitrary p2_i
% [x1,   x2,   ..., xk]     % but each landmark is associated (index-wise) to the relative p1_i 

% we will use a struct 'S' with fields p and X
k = width(S0.p); % number of matches
S.p = zeros(2,k); % 2d coordinates
S.X = zeros(3,k); % 3d landmarks coordinates

pointTracker = vision.PointTracker;
points1 = S0.p';
initialize(pointTracker,points1,img0)
setPoints(pointTracker,points1); 
[points2,points2_validity] = pointTracker(img1);

S.p = points2(points2_validity>0,:);
S.X = S0.X(:,points2_validity>0);

% calculate pose using p3p and ransac
[R_C_W, t_C_W, best_inlier_mask] = p3pRansac(S.p', S.X, K);

% cut the list of keypoints-landmark deleting outliers
S.p = points2(best_inlier_mask>0,:);
S.X = S.X(:,best_inlier_mask>0);

T_C_W = [R_C_W,t_C_W];
T_w_c = inv(T_C_W);

end


% for sol_idx = 1 : 4 : 16
% poses = p3pFitFunc(landmark_sample, normalized_bearings,sol_idx);
% %%%%%%%% TODO implement the metric to compute distance
% maxDistance = 10;
% [model,inlierIdx] = ransac(S.p,p3pFitFunc,distFcn,3,maxDistance); 
% n_inliers = lenght(inlierIdx>0);
% 
% if n_inliers > max_n_inliers
%     max_n_inliers = n_inliers;
%     optimal_T_C_W = model;
%     inliers = S.p(inlierIdx>0);
% end
% end

% function pose = p3pFitFunc(landmark_sample, normalized_bearings,sol_idx)
% poses = real(p3p(landmark_sample, normalized_bearings));
% pose = [poses(:,idx+1:idx+3),poses(:,sol_idx)];
% end
% 
% function distance = p3pModelDistance(model, data, corresponding_landmarks)
% R_C_W_guess = model(:,1:3);
% t_C_W_guess = model(:,end);
% projected_points = projectPoints((R_C_W_guess(:,:) * corresponding_landmarks) + repmat(t_C_W_guess(:,:),...
%     [1 size(corresponding_landmarks, 2)]), K);
% 
% difference = matched_query_keypoints - projected_points;
% errors = sum(difference.^2, 1);
% end


