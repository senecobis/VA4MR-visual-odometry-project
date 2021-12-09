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

% Made as part of the programming assignement
% for Vision Algoritms for Mobile Robotics course, fall 2021. ETH Zurich
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [p1_1, p1_2, ..., p1_k] % association matrices
% [p2_4, p2_9, ..., p2_k-5] % p1_1 can be associated with an arbitrary p2_i
% [x1,   x2,   ..., xk]     % but each landmark is associated (index-wise) to the relative p1_i 

% we will use a struct 'S' with fields p and X
k = width(S0.p); % number of matches
S.p = zeros(2,k); % 2d coordinates
S.X = zeros(3,k); % 3d landmarks coordinates
k = width(S0.C);
S.C = zeros(2,k);
S.F = zeros(2,k);
S.T = zeros(12,k);

pointTracker = vision.PointTracker;
points0 = S0.p';
%eccolo = [];
%eccolo = find(points1 < 0)
%fprintf('numero elementi negativi: %d', length(eccolo));
%%%%%%%%%%%%%%%%%%%% non capisco perchè ci va l'abs
points0 = abs(points0);
initialize(pointTracker,points0,img0)
setPoints(pointTracker,points0); 
[points1,points1_validity] = pointTracker(img1);
% eccolo = [];
% eccolo = find(points1 < 0);
% fprintf('numero elementi negativi: %d', length(eccolo));
%%%%%%%%%%%%%%%%%%%% il problema è che pointTracker ritorna cordinate
%%%%%%%%%%%%%%%%%%%% negative WTF!!!!! Forse bisogna passare da (x,y) a px
%%%%%%%%%%%%%%%%%%%% coordinates
S.p = points1(points1_validity>0,:);
S.X = S0.X(:,points1_validity>0);
%così s.p rimane piccolo
% if max(size(S.p)) > 401
%     S.p = S.p(1:400,:);
%     S.X = S.X(:,1:400);
%     points1 = S.p;
% end
%%%%%%%%%%%%% calculate pose using p3p and ransac
%fprintf('numero di punti in p3p %d:\n', height(S.p));
[R_C_W, t_C_W, best_inlier_mask] = p3pRansac(S.p', S.X, K);
%fprintf('matches ransac p3p:%d\n',nnz(best_inlier_mask));
%   fprintf('inliers p3p: %d',nnz(best_inlier_mask));
% R_C_W
% t_C_W

% cut the list of keypoints-landmark deleting outliers

S.p = points1(best_inlier_mask>0,:);
S.p = S.p';
S0.p = points0(best_inlier_mask>0,:);
S0.p = S0.p';
%size(S.p)
S.X = S.X(:,best_inlier_mask>0);
S.X = S0.X(:,best_inlier_mask>0);

T_C_W = [R_C_W,t_C_W; 0 0 0 1];
T_w_c = inv(T_C_W);
T_w_c = T_w_c(1:3,:);

%%%%%%%%%%%% printo le frames
printRelatuvePose = 0;
if printRelatuvePose
    figure(1)
    hold on
    plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
    text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');
    center_cam2_W = T_w_c * [0 0 0 1]';
    plotCoordinateFrame(T_w_c(1:3,1:3),T_w_c(1:3,4), 0.8);
    text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');
    axis equal
    rotate3d on;
    grid
    title('Cameras relative poses')
end
%%%%%%%%%%%%


S = extractKeyframes(S0, S, T_C_W(1:3,:), img0, img1, K);
S0.C = S.C;

end