function [S] = extractKeyframes(S0, S, T_C_W, img0, img1, K)
%EXTRACTKEYFRAMES takes as input and S.p,S.X as input. 
%It must return the whole state of the current frame: S.p,S.x,S.C,S.F,S.T.

% Setup
    T_C_W_hom = [T_C_W; 0 0 0 1];
    T_w_c = inv(T_C_W_hom);
    T_w_c = T_w_c(1:3,:);


% Create the point tracker
    tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);
    
% Initialize the point tracker
    initialize(tracker, S0.C', img0);
% Track the points
    [imagePoints1, validIdx] = step(tracker, img1);
    %matchedPoints0 = S0.C(:,validIdx);
    %elimino dallo stato i candidate keypoints non tracciati
    S.C = imagePoints1(validIdx, :);
    S.F = S0.F(: ,validIdx);
    S.T = S0.T(: ,validIdx);
    %aggiungo allo stato i nuovi candidate keypoints presenti in S.p
    for candidate_idx = 1:width(S0.p)
        if ~ismember(S0.p(candidate_idx),S.C)
            S.C(:,end+1) = S0.p(candidate_idx);
            S.F(:,end+1) = S0.p(candidate_idx);
            S.T(:,end+1) = reshape(T_w_c,[12,1]);
        end            
    end   
    
% Triangulate a point cloud using the final transformation (R,T)
% since we know the T_w_ci fr each component of S.C we can triangulate as a
% stereo camera between the two poses to get X_c1 and transform them in W
% later
    for candidate_idx = 1:height(S0.C)
        p0 = S0.F(:,candidate_idx);
        p0 = [p0; 1];
        bearing_0 = K\p0;
        p1 = S0.C(:,candidate_idx);
        p1 = [p1; 1];
        bearing_1 = K\p1;
        angolo = acos(dot(bearing_0,bearing_1)/(norm(bearing_0)*norm(bearing_1)));
        if angolo*pi/180 > 5
            M0 = K * reshape(S.T(:,candidate_idx),[3,4]);
            M1 = K * T_C_W;
            NewLandmarks = linearTriangulation(p0,p1,M0,M1);
            S.X = NewLandmarks(1:3,:);
        end
    end
% forse l'update dei landmarks potrei farlo solo per i nuovi aggiunti ma ho
% troppo sonno
end

