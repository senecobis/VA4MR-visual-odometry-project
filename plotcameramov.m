function t_n = plotcameramov(t,img, keypoints_img, t_prec,i)

t_n = abs(t_prec) + abs(t);

figure(3)

subplot(2,3,1)
plot(i, t_n(1), '.'); hold on
axis tight
subplot(2,3,2)
plot(i, t_n(2), '.'); hold on
subplot(2,3,3)
plot(i, t_n(3), '.'); hold on

subplot(2,3,5)
imshow(img,[]); hold on
plot(keypoints_img(1,:), keypoints_img(2,:),'x');
hold off
