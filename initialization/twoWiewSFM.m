function [T, matchedPoints0, matchedPoints1, landmarks] = twoWiewSFM(img0,img1,K,params)
    figures = 0;

    % Detect feature points
    %imagePoints0 = detectMinEigenFeatures(img0, 'MinQuality', 0.1);
    imagePoints0 = detectHarrisFeatures(img0, 'MinQuality', params.feature_quality,'FilterSize', params.filt_size);

    if figures
        % Visualize detected points
        figure
        imshow(img0, 'InitialMagnification', 50);
        title('200 Strongest Corners from the First Image');
        hold on
        plot(selectStrongest(imagePoints0, 200));
    end
    
    % Create the point tracker
    tracker = vision.PointTracker('MaxBidirectionalError', params.lambda, ...
                                   'NumPyramidLevels', params.num_pyr_levels, ...
                                   'BlockSize', params.bl_size, ...
                                   'MaxIterations', params.max_its);

    % Initialize the point tracker
    p0 = imagePoints0.Location;
    initialize(tracker, p0, img0);

    % Track the points
    [imagePoints2, validIdx] = step(tracker, img1);
    matchedPoints0 = p0(validIdx, :);
    matchedPoints1 = imagePoints2(validIdx, :);

    % Estimate the fundamental matrix
    [F, inliers] = estimateFundamentalMatrix(matchedPoints0, matchedPoints1,'Confidence', 99.99);

    matchedPoints0 = matchedPoints0(inliers,:);
    matchedPoints1 = matchedPoints1(inliers,:);

    [R,t,validPointsFraction] = relativeCameraPose(F, params.cam, matchedPoints0, matchedPoints1);

    if validPointsFraction < 0.5
    warning('[relativeCameraPose] ERROR: relative pose is invalid %f', valid_points_fraction);
    end

    %triangulate points
    T = [R, t.'];
    p0_ho = [matchedPoints0, ones(height(matchedPoints0),1)].';
    p1_ho = [matchedPoints1, ones(height(matchedPoints1),1)].';
    landmarks = linearTriangulation(p0_ho, p1_ho, K*eye(3,4),K*T);


    if figures
        % Display inlier matches
        figure
        showMatchedFeatures(img0, img1, matchedPoints0, matchedPoints1);
        title('Epipolar Inliers');
    end
    
    
    %landmarks = pointCloud(img0, p0_ho, p1_ho, K, T, 0);

end