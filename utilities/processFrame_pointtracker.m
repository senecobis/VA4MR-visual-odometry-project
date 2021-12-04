function [S, T_w_c] = processFrame(S0, img0, img1)
    % The continuous VO pipeline is the core component of the proposed VO implementation. 
    % Its responsibilities are three-fold:
    % 1. Associate keypoints in the current frame to previously triangulated landmarks.
    % 2. Based on this, estimate the current camera pose.
    % 3. Regularly triangulate new landmarks using keypoints not associated to previously triangulated landmarks.
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Function to process each frame, handler of the continuos VO pipeline
    % input --> the 2 images RGB or GRAYSCALE, and the previous state
    % output --> S : the state (a struct) containing a [2,k] matrix rapresenting 2d keypoint coordinates and a 
    % [3,k] matrix rapresenting 3d landmarks coordinates. 
    % T_w_c : the transformation from the world to cameras frame of img1 with respect to img0
    
    % Made as part of the programming assignement for Vision Algoritms for Mobile Robotics course, autumn 2021. ETH Zurich
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % [p1_1, p1_2, ..., p1_k] % association matrices
    % [p2_4, p2_9, ..., p2_k-5] % p1_1 can be associated with an arbitrary p2_i
    % [x1,   x2,   ..., xk]     % but each landmark is associated (index-wise) to the relative p1_i 
    % we will use a struct 'S' with fields p and X

    pointTracker = vision.PointTracker;
    initialize(pointTracker,S0.p,img0);
    [points, isFound] = step(pointTracker,img1);
    validPoints = points(isFound,:);
    oldInliers = S0.p(isFound, :);
    
    fprintf('number of keypoints inliers %d', nnz(isFound));
    
    figure(3);
    imshow(img1);
    out = insertMarker(img1,validPoints,'+','Color','red');
    imshow(out);
    
    setPoints(pointTracker, validPoints); 
    T_w_c = zeros(3,4);
    S.p = validPoints;
    S.X = S0.X(isFound == 1,:);


end