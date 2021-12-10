function [T_I_C_new,P] = DisplayTrajectory(T_I_C, T_W_C, S, img, positions)
% function to plot trajectory
% T_I_C : is the absolute transformation of the camera from inertial frame
% and it is the product of all the pose up until the current frame
% position : 

T_I_C_new = [T_I_C; 0 0 0 1] * [T_W_C; 0 0 0 1];
T_I_C_new = T_I_C_new(1:3,1:end);

P = [positions; T_I_C_new(1:3,4)'];

figure(7)
subplot(2,2,1)
imshow(img,[]); hold on
plot(S.p(1,:),S.p(2,:),'o');
subplot(2,2,2)
plot3(S.X(1,:),S.X(2,:),S.X(3,:),'o')

subplot(2,2,[3 4])
plot(P(:,1), P(:,3)); hold on
plot(S.X(1,:),S.X(3,:),'o');
ylim([P(end,3)-50, P(end,3)+50]);
axis equal


end