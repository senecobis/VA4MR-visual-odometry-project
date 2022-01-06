function [S, T_w_c1] = processFrame(S0, img0, img1, K,params)
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

% Made as part of the programming assignement
% for Vision Algoritms for Mobile Robotics course, fall 2021. ETH Zurich
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [p1_1, p1_2, ..., p1_k] % association matrices
% [p2_4, p2_9, ..., p2_k-5] % p1_1 can be associated with an arbitrary p2_i
% [x1,   x2,   ..., xk]     % but each landmark is associated (index-wise) to the relative p1_i 

% we will use a struct 'S' with fields p and X
% k = width(S0.p); % number of matches
% S.p = zeros(2,k); % 2d coordinates
% S.X = zeros(3,k); % 3d landmarks coordinates
%k = width(S0.C);
S.C = S0.C;
S.F = S0.F;
S.T = S0.T;
S.HoP = S0.HoP;
S.HoL = S0.HoL;
figures = 0;

S0.p = round(S0.p);
pointTracker = vision.PointTracker('MaxBidirectionalError', params.lambda, ...
                                   'NumPyramidLevels', params.num_pyr_levels, ...
                                   'BlockSize', params.bl_size, ...
                                   'MaxIterations', params.max_its);
initialize(pointTracker,S0.p.',img0)
setPoints(pointTracker,S0.p.'); 

%Roba per non dover avere troppi keypoints -Lollo
[trackedKeypoints, isTracked] = step(pointTracker, img1);

S.p = trackedKeypoints(isTracked,:).';
S.X = S0.X(:,isTracked);

% estimateWorldCameraPose is a matlab func that requires double or single inputs
S.p = double(S.p);
S.X = double(S.X);

fprintf('numero keypoints:%d  \n',length(S.p));

% Estimate the camera pose in the world coordinate system
[R, T, best_inlier_mask, status] = estimateWorldCameraPose(S.p.', S.X.', params.cam, ...
                                'MaxNumTrials', params.max_num_trials, ...
                                'Confidence', params.conf, ...
                                'MaxReprojectionError', params.max_repr_err);

% cut the list of keypoints-landmark deleting outliers
S.p = S.p(:,best_inlier_mask);
S.X = S.X(:,best_inlier_mask);

% Combine orientation and translation into a single transformation matrix
T_w_c1 = [R, T.'; 0 0 0 1];

% Extract new keyframes
S = extractKeyframes(S, T_w_c1, img0, img1, K, params);

end