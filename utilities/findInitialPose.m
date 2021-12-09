function [R,T,inliers] = findInitialPose(p1, p2, K)
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function to bootstraps the initial camera poses.
% input --> p1 and p2 are two matrices N x 2 where N are the matched
% points, in first and second column we find the 2d coordinates of this points
% output --> the transoformation from the 2 cameras frames
% Made  as part of the programming assignement
% for Vision Algoritms for Mobile Robotics course, autumn 2021. ETH Zurich
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% imageSize = size(img1);
% focalLength = [331.3700, 369.5680];
% principalPoint = [320, 240];
% intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);

K1 = K;
p1_int = cast(p1,"int32");
p2_int = cast(p2,"int32");
% E = estimateEssentialMatrix(p1,p2,intrinsics);
% p1 e p2 hanno valori intermedi alle coordinate dei px e bisogna
% arrotondare

[F, inliers] = estimateFundamentalMatrix(p1_int,p2_int,'Method','RANSAC',...
     'NumTrials',5000,'DistanceThreshold',1e-2,'Confidence',99.9);
% Così usiamo ransac e leviamo le schifezze di match che escono ogni tanto
E = inv(K)'\ F * K;
[R,u] = decomposeEssentialMatrix(E);
num_keyp = size(p1(inliers,:),1);
p1_ho = [p1(inliers,:), ones(num_keyp,1)]';
p2_ho = [p2(inliers,:), ones(num_keyp,1)]';
[R,T] = disambiguateRelativePose(R,u,p1_ho,p2_ho,K1,K1);

end