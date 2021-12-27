function [params] = getParams(dataset)
% save all parameters to a parameters struct
%   :param dataset: string, the dataset used, eg. 'kitti', 'malaga', 'parking',
%       'ascento'
%   :return params: struct, the parameters
%
% NOTE: add your parameters here. If the parameter is specific for a
% dataset add it again in the 'dataset specific parameters' section and
% make sure to add a comment in the default section that it was overriden.
% For an example see the parameter 'bootstrap_frames' which is overriden
% for all datasets

%% General parameters
params.kitti_path = 'data/kitti';
params.malaga_path = 'data/malaga';
params.parking_path = 'data/parking';

%% Initialization parameters
params.bootstrap_frames = [1, 3];   % overriden for all datasets
params.uniq = false;
params.filt_size = 5;
params.max_ratio = 0.5;   

% keypoint detection and maching
params.method = 2; %0 = Harris, 1 = Surf; %2 Shi-Tomasi
params.feature_quality = 1e-4;
params.n_keypoints = 500;
params.descriptor_size = 11;        % has to be odd
assert(mod(params.descriptor_size, 2)==1)
params.matching_mode = 'patch_matching'; %'patch_matching' or 'klt'
params.max_num_keypoints = 100; %aumentare per più precisione -Lollo


%% Dataset specific parameters
%% params for KITTI
if dataset == 0
    params.bootstrap_frames = [1,3];
    
    % Matching keypoints
    params.n_keypoints = 1200; % num of keypoints at each frame, it must be constant
    params.uniq = true;
    params.max_ratio = 0.97;
    params.feature_quality = 1e-4;
    params.filt_size = 3;
    
    % 8 point algorithm
    params.eightp_num_trials = 64000;
    params.eightp_dist_threshold = 0.001;
    params.eightp_confidence = 100;
    
    % Continuous operation parameters
    params.min_angle = 2;               % minimum angle for triangulating
    params.new_cand_kpt_threshold = 3;  % threshold for selecting new 
                                        % candidate keypoints
    % KLT parameters
    params.lambda = 2;                  % maximum bidirectional error
    params.num_pyr_levels = 6;
    params.bl_size = [31, 31];
    params.max_its = 2000;

    % P3P parameters
    params.max_num_trials = 100000;
    params.conf = 99;
    params.max_repr_err = 2;
    
    % Triangulation of new landmarks parameters
    params.strong_to_uniform_kp_ratio = 0.13;
    
%% params for MALAGA
elseif dataset == 1
    params.bootstrap_frames = [1,3];
    
    % Matching keypoints
    params.n_keypoints = 1000;
    params.uniq = true;
    params.max_ratio = 0.9;
    params.feature_quality = 1e-4;
    params.filt_size = 3;
    
    % 8 point algorithm
    params.eightp_num_trials = 64000;
    params.eightp_dist_threshold = 0.001;
    params.eightp_confidence = 99.99;
    
    % Continuous operation parameters
    params.min_angle = 2;               % minimum angle for triangulating
    params.new_cand_kpt_threshold = 3;  % threshold for selecting new 
                                        % candidate keypoints
    % KLT parameters
    params.lambda = 2;                  % maximum bidirectional error
    params.num_pyr_levels = 5;
    params.bl_size = [31, 31];
    params.max_its = 40;

    % P3P parameters
    params.max_num_trials = 100000;
    params.conf = 99;
    params.max_repr_err = 1;
    
    % triangulation of new landmarks parameters
    params.strong_to_uniform_kp_ratio = 0.16;

%% params for PARKING
elseif dataset == 2
    params.bootstrap_frames = [1,5];
    
    % Matching keypoints
    params.n_keypoints = 200;
    params.uniq = true;
    params.max_ratio = 0.9;
    
    % 8 point algorithm
    params.eightp_num_trials = 32000;
    params.eightp_dist_threshold = 0.0001;
    params.eightp_confidence = 81;
    
    % Continuous operation parameters
    params.min_angle = 2;               % minimum angle for triangulating
    params.new_cand_kpt_threshold = 3;  % threshold for selecting new 
                                        % candidate keypoints
    % KLT parameters
    params.lambda = 2;                  % maximum bidirectional error
    params.num_pyr_levels = 6;
    params.bl_size = [31, 31];
    params.max_its = 200;

    % P3P parameters
    params.max_num_trials = 16000;
    params.conf = 99.973;
    params.max_repr_err = 0.6345;
    
    % triangulation of new landmarks parameters
    params.strong_to_uniform_kp_ratio = 0.08;

end