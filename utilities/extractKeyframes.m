function [S,CandidateTracker] = extractKeyframes(S0, S, T_C_W, img0, img1, K, CandidateTracker)
%EXTRACTKEYFRAMES takes as input and S.p, S.X as input. 
%It must return the whole state of the current frame: S.p,S.x,S.C,S.F,S.T.

% Setup
    T_C_W_hom = [T_C_W; 0 0 0 1];
    T_w_c = inv(T_C_W_hom);
    T_w_c = T_w_c(1:3,:);
    
%% Traccio i keypoints nella sequenza di S.C

[pointstrack, pointstrack_validity] = CandidateTracker(img1);
S.C = pointstrack(pointstrack_validity>0,:)';


max_angolo = 5;
%fprintf('%d\n',width(S.C));
    for candidate_idx = 1:max(size(S.C))
        p_orig = S.F(:,candidate_idx);
        p_orig = [p_orig; 1];
        bearing_orig = K\p_orig;
        p_curr = S.C(:,candidate_idx);
        p_curr = [p_curr; 1];
        bearing_curr = K\p_curr;
        angolo = acos(dot(bearing_orig,bearing_curr)/(norm(bearing_orig)*norm(bearing_curr)));
        % alternative to calculate angle, is the same
%         angolo1 = atan2(norm(cross(bearing_orig,bearing_curr)), dot(bearing_orig,bearing_curr));
%         diff = angolo - angolo1
        angolo_grad = angolo*180/pi;
%         if angolo_grad > max_angolo
%             max_angolo = angolo_grad;
%         end
        %%%%%%%% need to refine threshold %%%%%%%%
        if abs(angolo_grad) > max_angolo
            M0 = K * reshape(S.T(:,candidate_idx),[3,4]);
            M1 = K * T_w_c;
            NewLandmarks = linearTriangulation(p_orig,p_curr,M0,M1);
            S.X(:,end+1) = NewLandmarks(1:3,:);
            S.p(:,end+1) = S.C(:,candidate_idx);
        end
    end


%% Trovo nuovi candidate keypoints da aggiungere in S.C
% Detect feature points
    imagePoints0 = detectMinEigenFeatures(img1, 'filtersize',5,'MinQuality', 0.0001);
    imagePoints0 = selectStrongest(imagePoints0,1000);
    candidate_keypoints = imagePoints0.Location'; %2xM
% Ora devo aggiungere questi nuovi candidati a S.C se non sono già
% presenti in S.C o S.p
for new_candidate_idx = 1:size(max(candidate_keypoints))
    if ~ismember(candidate_keypoints(new_candidate_idx),S.F) & ~ismember(candidate_keypoints(new_candidate_idx),S.p)
        S.C(:,end+1) = candidate_keypoints(new_candidate_idx);
        S.F(:,end+1) = candidate_keypoints(new_candidate_idx);
        S.T(:,end+1) = reshape(T_w_c,[12,1]);
        fprintf('Porcodiiii\n');
    else
        fprintf('false\n');
    end
end

%updato il tracker
setPoints(CandidateTracker, S.C');

end










