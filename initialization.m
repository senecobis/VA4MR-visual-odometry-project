function [T, keypoints_img1, keypoints_img2, landmarks] = initialization(img1, img2, K)

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function to bootstraps the initial camera poses.
% input --> the 2 images as GRAYSCALE
% output --> the transoformation from the 2 cameras frames
% Made by senecobis :rpellerito@ethz.ch as part of the programming assignement
% for Vision Algoritms for Mobile Robotics course, autumn 2021. ETH Zurich
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% first we have to find the keypoints of the two image and match it
    % we will do so by using harris (o shi-tommasi) scores 
    % than we do non maximum suppression and select the highest scores
    % than we find keypoints for each image, the return should be 
    % 2 x num_keypoints containing 2d coordinates of each keypoint for
    % the considered image

addpath('exercises_fun\')

[p1,p2] = matchKeypoints(im2gray(img1),im2gray(img2));
keypoints_img1 = p1;
keypoints_img2 = p2;
[R,t] = findInitialPose(p1, p2, K);
T = [R,t];
p1_ho = [p1, ones(length(p1),1)]';
p2_ho = [p2, ones(length(p2),1)]';
landmarks = linearTriangulation(p1_ho, p2_ho,K*[eye(3),zeros(3,1)],K*T);
% landmarks = pointCloud(img1, img2, p1_ho, p2_ho, K, T); 

end
