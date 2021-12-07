function [T, keypoints_img0, keypoints_img1, landmarks] = twoWiewSFM(img0,img1,K)
    % options
    figures = 0;
    Shi_Thomasi = 0;
    surf = 1 - Shi_Thomasi;
    
    % change numiter to verify accuracy of SFM
    numiter = 1;
    for i = 1:numiter
        if i > 1
           fprintf('%d\n',i);
        end
        % Detect feature points
        if Shi_Thomasi
            imagePoints0 = detectMinEigenFeatures(img0, 'FilterSize',5 ,'MinQuality', 0.001);
        end

        if surf 
            imagePoints0 = detectSURFFeatures(img0,'MetricThreshold',1000);
        end

        if figures
            % Visualize detected points
            figure
            imshow(img0, 'InitialMagnification', 50);
            title('200 Strongest Corners from the First Image');
            hold on
            plot(selectStrongest(imagePoints0, 200));
        end

        % Create the point tracker
        tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);

        % Initialize the point tracker
        p0 = imagePoints0.Location;
        initialize(tracker, p0, img0);

        % Track the points
        [imagePoints2, validIdx] = step(tracker, img1);
        matchedPoints0 = p0(validIdx, :);
        matchedPoints1 = imagePoints2(validIdx, :);

        if figures
            % Visualize correspondences
            figure
            showMatchedFeatures(img0, img1, matchedPoints0, matchedPoints1);
            title('Tracked Features');
        end

        % Estimate the fundamental matrix
        [F, inliers] = estimateFundamentalMatrix(round(matchedPoints0),round(matchedPoints1),'Method','RANSAC',...
                                                 'NumTrials',5000,'DistanceThreshold',1e-4,'Confidence',99.99); 
        E = inv(K)'\ F * K;
        [Rots,u] = decomposeEssentialMatrix(E);
        num_keyp = size(matchedPoints0(inliers,:),1);
        p0_ho = [matchedPoints0(inliers>0,:), ones(num_keyp,1)]';
        p1_ho = [matchedPoints1(inliers>0,:), ones(num_keyp,1)]';
        [R,t] = disambiguateRelativePose(Rots,u,p0_ho,p1_ho,K,K);

        % Find epipolar inliers
        keypoints_img0 = matchedPoints0(inliers>0, :);
        keypoints_img1 = matchedPoints1(inliers>0, :);

        if figures
            % Display inlier matches
            figure
            showMatchedFeatures(img0, img1, keypoints_img0, keypoints_img1);
            title('Epipolar Inliers');
        end

        %triangulate points
        T = [R, t];
        landmarks = pointCloud(img0, img1, p0_ho, p1_ho, K, T);
    end
end