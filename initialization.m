function T = initialization(img1, img2, K)

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function to bootstraps the initial camera poses.
% input --> the 2 images as GRAYSCALE
% output --> the matches locations in the 2 cameras frames
% Made by senecobis as part of the programming assignement for Vision
% Algoritms for Mobile Robotics course, autumn 2021. ETH Zurich
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% first we have to find the keypoints of the two image and match it
    % we will do so by using harris (o shi-tommasi) scores 
    % than we do non maximum suppression and select the highest scores
    % than we find keypoints for each image, the return should be 
    % 2 x num_keypoints containing 2d coordinates of each keypoint for
    % the considered image

addpath('exercises_fun\')

[p1,p2] = matchKeypoints(im2gray(img1),im2gray(img2))
[R,t] = findInitialPose(p1, p2, K);
T = [R,t];

end

function [R,T] = findInitialPose(p1, p2, K)
% p1 and p2 are two matrices N x 2 where N are the matched points and in
% first and second column we find the 2d coordinates of this points
% imageSize = size(img1);
% focalLength = [331.3700, 369.5680];
% principalPoint = [320, 240];
% intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
% K = K;
% E = estimateEssentialMatrix(p1,p2,intrinsics);
F = estimateFundamentalMatrix(p1,p2);
E = K' * F * K;
[R,u] = decomposeEssentialMatrix(E);
p1_ho = [p1, ones(length(p1))];
p2_ho = [p2, ones(length(p2))];
[R,T] = disambiguateRelativePose(R,u,p1_ho,p2_ho,K1,K1);

end
