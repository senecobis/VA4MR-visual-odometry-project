function [S] = extractKeyframes(S0, S, T_C_W, img0, img1, K)
%EXTRACTKEYFRAMES takes as input and S.p,S.X as input. 
%It must return the whole state of the current frame: S.p,S.x,S.C,S.F,S.T.

% Setup
    T_C_W_hom = [T_C_W; 0 0 0 1];
    T_w_c = inv(T_C_W_hom);
    T_w_c = T_w_c(1:3,:);
    
%% Traccio i keypoints nella sequenza di S.C
% Create the point tracker
    tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);
    
% Initialize the point tracker
    initialize(tracker, S0.C', img0);
    
% Track the points
    [imagePoints1, validIdx] = step(tracker, img1);
    %matchedPoints0 = S0.C(:,validIdx);
    %elimino dallo stato i candidate keypoints non tracciati
    S.C = imagePoints1(validIdx, :);
    S.C = S.C';
    S.F = S0.F(: ,validIdx);
    S.T = S0.T(: ,validIdx);
    
%%%%%%%%%%%%%%%%%   debug
% ho tolto questo pezzo perchè penso sia quello che faccio in right 24-30
%     %aggiungo allo stato i nuovi candidate keypoints presenti in S.p
%     for candidate_idx = 1:width(S0.p)
%         if ~ismember(S0.p(candidate_idx),S.C)
%             S.C(:,end+1) = S0.p(candidate_idx);
%             S.F(:,end+1) = S0.p(candidate_idx);
%             S.T(:,end+1) = reshape(T_w_c,[12,1]);
%         end            
%     end  

% Update S.p and S.X with the keypoints that pass the bearing angle test
% S0
% S
max_angolo = 0;
%fprintf('%d\n',width(S.C));
    for candidate_idx = 1:width(S.C)
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
        angolo_grad = angolo*pi/180;
        if angolo_grad > max_angolo
            max_angolo = angolo_grad;
        end
        %%%%%%%% need to refine threshold %%%%%%%%
        if angolo_grad > 1
            fprintf('update angolo')
            M0 = K * reshape(S.T(:,candidate_idx),[3,4]);
            M1 = K * T_w_c;
            NewLandmarks = linearTriangulation(p_orig,p_curr,M0,M1);
            S.X(:,end+1) = NewLandmarks(1:3,:);
            S.p(:,end+1) = S.C(:,candidate_idx);
        end
    end
%fprintf('angolo massimo: %d', max_angolo);
% forse l'update dei landmarks potrei farlo solo per i nuovi aggiunti ma ho
% troppo sonno
% S0
% S

%% Trovo nuovi candidate keypoints da aggiungere in S.C
% Detect feature points
    imagePoints0 = detectMinEigenFeatures(img1, 'MinQuality', 0.1);
    imagePoints0 = selectStrongest(imagePoints0,200);
    candidate_keypoints = imagePoints0.Location'; %2xM
% Ora devo aggiungere questi nuovi candidati a S.C sse non sonon già
% pressenti in S.C o S.p
    for new_candidate_idx = 1:width(candidate_keypoints)
        if ~ismember(candidate_keypoints(new_candidate_idx),S.C) & ~ismember(candidate_keypoints(new_candidate_idx),S.p)
            S.C(:,end+1) = candidate_keypoints(new_candidate_idx);
            S.F(:,end+1) = candidate_keypoints(new_candidate_idx);
            S.T(:,end+1) = reshape(T_w_c,[12,1]);
        else
            fprintf('false\n');
        end
    end


end

