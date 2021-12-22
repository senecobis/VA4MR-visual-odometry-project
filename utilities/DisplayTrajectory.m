function [S] = DisplayTrajectory(T_w_c0, img, S, i)
% function to plot trajectory


%% Init
cont = S.cont; %numero di nuovi keypoints
T_I_C_new = T_w_c0(1:3,1:end); %Posizione camera
%positions = S.HoP;  %aggiorno la history of positions
S.HoP(end+1,:) = T_I_C_new(1:3,4)';
newpos = S.HoP;

%% Immagine con i match
figure(3)
subplot(2,2,1)
imshow(img,[]); hold on
plot(S.p(1,1:end-cont), S.p(2,1:end-cont),'ob');
plot(S.p(1,end-cont:end), S.p(2,end-cont:end),'or');
legend('Keypoints in utilizzo','Nuovi Keypoints')
title('Keypoints')
hold off

%% 3d Points
subplot(2,2,2)
plot3(S.X(1,:),S.X(2,:),S.X(3,:),'o'), hold on
if 1 % Se si vuole pure vedere la traiettoria in 3D
    plot3(newpos(:,1), newpos(:,2), newpos(:,3), 'LineWidth', 3);
end
if 1 %Se si vuole anche plottare la posizione della camera
    camera_name = append('camera', string(i));
    center_cam2_W = T_w_c0(1:3,end);
    plotCoordinateFrame(T_w_c0(1:3,1:3),T_w_c0(1:3,4), 0.8);
    text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,camera_name,'fontsize',10,'color','k','FontWeight','bold');
    axis equal
    rotate3d on;
    grid
    title('Cameras relative poses and 3d landmarks')
end

hold off



%% Grafico movimento visto dall'alto
subplot(2,2,[3 4])
plot(newpos(:,1), newpos(:,3), 'LineWidth', 5); hold on
plot(S.X(1,:),S.X(3,:),'o');
ylim([newpos(end,3)-10, newpos(end,3)+100]);
xlim([newpos(end,1)-3, newpos(end,1)+30]);
axis equal
title('Cameras trajectory and Landmarks (upside view)')
hold off
end