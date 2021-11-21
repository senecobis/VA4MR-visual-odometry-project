function [R,T] = findInitialPose(p1, p2, K)
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function to bootstraps the initial camera poses.
% input --> p1 and p2 are two matrices N x 2 where N are the matched
% points, in first and second column we find the 2d coordinates of this points
% output --> the transoformation from the 2 cameras frames
% Made by senecobis :rpellerito@ethz.ch as part of the programming assignement
% for Vision Algoritms for Mobile Robotics course, autumn 2021. ETH Zurich
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% imageSize = size(img1);
% focalLength = [331.3700, 369.5680];
% principalPoint = [320, 240];
% intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
K1 = K;
% E = estimateEssentialMatrix(p1,p2,intrinsics);
F = estimateFundamentalMatrix(p1,p2);
E = K' * F * K;
[R,u] = decomposeEssentialMatrix(E);
p1_ho = [p1, ones(length(p1),1)]';
p2_ho = [p2, ones(length(p2),1)]';
[R,T] = disambiguateRelativePose(R,u,p1_ho,p2_ho,K1,K1);

end