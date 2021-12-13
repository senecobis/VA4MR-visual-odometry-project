function [S] = DisplayTrajectory(T_w_c0, img, S, cont)
% function to plot trajectory
% T_I_C : is the absolute transformation of the camera from inertial frame
% and it is the product of all the pose up until the current frame
% position : 

%T_I_C_new = [T_I_C; 0 0 0 1] * [T_W_C; 0 0 0 1];
% T_w_c0 = inv(T_w_c0);
T_I_C_new = T_w_c0(1:3,1:end);

positions = S.HoP;
S.HoP = [positions; T_I_C_new(1:3,4)'];
newpos = S.HoP;

figure(7)
subplot(2,2,1)
% Immagine con i match
imshow(img,[]); hold on
plot(S.p(1,1:end-cont),S.p(2,1:end-cont),'ob');
plot(S.p(1,end-cont:end),S.p(2,end-cont:end),'or');

hold off

%3d Points
subplot(2,2,2)
plot3(S.X(1,:),S.X(2,:),S.X(3,:),'o')

subplot(2,2,[3 4])
%grafico movimento
plot(newpos(:,1), newpos(:,3)); hold on
%plot(S.X(1,:),S.X(3,:),'o');
ylim([newpos(end,3)-50, newpos(end,3)+50]);
xlim([newpos(end,1)-50, newpos(end,1)+50]);
axis equal




end