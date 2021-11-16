function [P, X, C, F, T] = initialization(img1, img2)

% i return sono ancora non usati

% -Rob : scrivo ste descrizioni e commenti cosi le mettiamo nel file che
% vuole la scaramuccia

    % first we have to find the keypoints of the two image and match it
        % we will do so by using harris (o shi-tommasi) scores 
        % than we do non maximum suppression and select the highest scores
        % than we find keypoints for each image, the return should be 
        % 2 x num_keypoints containing 2d coordinates of each keypoint for
        % the considered image

addpath('exercises_fun\')

%% parameters ===> we should define global variables or a files cointaining it for this parameters
corner_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 300;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 4;

harris_scores_1 = harris(img1, corner_patch_size, harris_kappa);
keypoints_1 = selectKeypoints(harris_scores_1, num_keypoints, nonmaximum_supression_radius);

harris_scores_2 = harris(img2, corner_patch_size, harris_kappa);
keypoints_2 = selectKeypoints(harris_scores_2, num_keypoints, nonmaximum_supression_radius);

% database descriptors are descriptors from the first image
% query... are from the second image
database_descriptors = describeKeypoints(img1, keypoints_1, descriptor_radius);
query_descriptors = describeKeypoints(img2, keypoints_2, descriptor_radius);

matches = matchDescriptors(query_descriptors, database_descriptors, match_lambda);

plotCorrespondances(img2, keypoints_1, keypoints_2, matches)



end

function plotCorrespondances(img2, keypoints_1, keypoints_2, matches) 

figure(4);
imshow(img2);
hold on;
plot(keypoints_2(2, :), keypoints_2(1, :), 'rx', 'Linewidth', 2);
plotMatches(matches, keypoints_2, keypoints_1);


end

function [R,T] = findInitialPose(p1, p2, K)
% p1 and p2 are two matrices N x 2 where N are the matched points and in
% first and second column we find the 2d coordinates of this points

E = estimateEssentialMatrix(p1,p2,K);
[R,u] = decomposeEssentialMatrix(E);
[R,T] = disambiguateRelativePose(R,u,p1,p2,K1,K1);

end
