%% Init
close all
clear all
clc

%% Import
addpath('utilities/')

%% Setup
ds = 1; % 0: KITTI, 1: Malaga, 2: parking

if ds == 0
    % need to set kitti_path to folder containing "05" and "poses"
    kitti_path = 'kitti';
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/05.txt']);
    poses = reshape(ground_truth, [3,4,length(ground_truth)]);
    poses(:,:,5);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    % Path containing the many files of Malaga 7.
    malaga_path = 'malaga-urban-dataset-extract-07';
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    % Path containing images, depths and all...
    parking_path = 'parking';
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);     
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
else
    assert(false);
end

%% Bootstrap
% need to set bootstrap_frames
bootstrap_frames = [1 3];
if ds == 0
    img0 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
else
    assert(false);
end

%%%%%%%%%%%%%%%%%% testing on main 
[T_w_c, keypoints_img0, keypoints_img1, landmarks] = twoWiewSFM(img0,img1,K);
S0.p = keypoints_img0';
S0.X = landmarks(1:3,:);
%.C è una matrice 2xM con le coord. dei candidate keypoints (M = # candidates)
S0.C = keypoints_img0';
% .F è una matrice 2xM con le coord. dei candidate keypoints nel primo
% frame in cui sono stati estratti
S0.F = keypoints_img0';
% .T è una matrice 12xM in cui ogni colonna è la T_w_c del primo frame per
% ogni keypoint reshaped in colonna
S0.T = reshape(T_w_c,[12,1]).*ones(12,height(keypoints_img0));
                                       
%fprintf("ground truth")
prev_img = img0;
t_n = 0;

%% Continuous operation
range = (bootstrap_frames(2)+1):last_frame;
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end
    % here put functions to plot results : trajectorie, keypoints  and landmarks
    % firstly process frame needs an initialization of S0, according to the
    % dimension requested. This init can be done through initialization (by changing it)
    [S, T_w_c] = processFrame(S0, prev_img, image, K);
    t_n = plotcameramov(T_w_c(1:3,4), image, S.p, t_n, i);
    
    % Makes sure that plots refresh.    
    pause(0.1);    
    prev_img = image;
end

