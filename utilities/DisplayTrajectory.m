function T_I_C_new = DisplayTrajectory(T_I_C, T_W_C, keypoints)
% function to plot trajectory
% T_I_C is the absolute transformation of the camera from inertial frame
T_I_C_new = [T_I_C; 0 0 0 1] * [T_W_C; 0 0 0 1];
T_I_C_new = T_I_C_new(1:3,1:end);


figure(3)
subplot(2,3,1)
plot(T_I_C_new(1,end), T_I_C_new(2,end), '.'); hold on
xlabel('x')
ylabel('y')
axis tight
% subplot(2,3,2)
% plot(T_I_C_new(1,4), T_I_C_new(2,4), T_I_C_new(3,4), '.'); hold on
% xlabel('x')
% ylabel('y')
% zlabel('z')
% subplot(2,3,3)
% plot(i, t_n(3), '.'); hold on
% 
% subplot(2,3,5)
% imshow(img,[]); hold on
% if length(keypoints) > width(keypoints)
%     plot(keypoints(:,1), keypoints(:,2),'x');
% else
%     plot(keypoints(1,:), keypoints(2,:),'x');
% end

hold off



end