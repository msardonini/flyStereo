clear all; clc; close all
clear all; clc;

% data = dlmread('../build_docker/replay/run004/trajectory.txt');

data = dlmread('../replay_data/flight/11_21_20/run013/trajectory.txt');

% data = dlmread('/home/msardonini/mntNano/flyStereo/build/file.txt');
% data = dlmread('/home/msardonini/git/flyStereo/replay_data/flight/8_22_20/run051/trajectory.txt');


pose = data(:,1:16);
kalman_output = data(:,17:22);
R_imu = zeros(3, 3, size(data,1));
% Initialize the variables
Xform = repmat(eye(4), [1, 1, size(data,1)]);
points = zeros(4, size(data,1));
velocity = zeros(4, size(data,1));
bearing = zeros(3, size(data,1)); bearing(:,1) = [0 0 -1];
euler = zeros(3, size(data,1));
imu_euler = zeros(3, size(data,1));

running_xform = eye(4);
running_r_imu = eye(3);
for i = 1:size(data,1)
   Xform(:, :, i) = reshape(pose(i,:),[4,4])';
   running_r_imu = running_r_imu * reshape(data(i,23:31),[3,3])';
   R_imu(:, :, i) = running_r_imu;

%    running_xform = running_xform * Xform(:,:, i); %uncomment for running delta xforms
   running_xform = Xform(:,:, i); % uncomment for logged absolute xform

   euler(:,i) = R2Euler(running_xform);
   imu_euler(:,i) = R2Euler(R_imu(:,:,i));
   bearing(:, i) = running_xform(1:3, 1:3) * [0; 0; -1];
   points(:, i) = running_xform * [0; 0; 0; 1];
   if (i ~= 1)
       velocity(:,i) = points(:, i) - points(:, i - 1);
   end

end

figure
scatter3(points(1,:), points(2,:), points(3,:))
hold on
% Orientation of the camera
quiver3(points(1,:), points(2,:), points(3,:), bearing(1,:), bearing(2,:),...
    bearing(3,:))
% Velocity
% quiver3(points(1,:), points(2,:), points(3,:), velocity(1,:), velocity(2,:),...
%     velocity(3,:))
xlabel('X axis'); ylabel('Y axis'); zlabel('Z axis'); axis equal



% figure
% scatter3(points(1,:), points(2,:), points(3,:))
% hold on
% Orientation of the camera
% bsize = size(bearing,2);
% quiver3(zeros(bsize,1)', zeros(bsize,1)', zeros(bsize,1)', bearing(1,:),...
%     bearing(2,:), bearing(3,:))
% Velocity
% quiver3(points(1,:), points(2,:), points(3,:), velocity(1,:), velocity(2,:),...
%     velocity(3,:))
% xlabel('X axis'); ylabel('Y axis'); zlabel('Z axis'); axis equal

%%
figure
plot(unwrap(euler'))
hold on
plot(unwrap(imu_euler'))
title('Euler Angles')
legend('x vio', 'y vio', 'z vio', 'X imu', 'Y imu', 'Z imu')


% Figure to show the weird linear relationship between distance travelled
% and estimated rotation
% figure
% plot(kalman_output(:,1),unwrap(euler(1,:)))
%


figure
scatter3(kalman_output(:,1),kalman_output(:,3), kalman_output(:,5))
hold on
quiver3(kalman_output(:,1),kalman_output(:,3), kalman_output(:,5), ...
    10*kalman_output(:,2),10*kalman_output(:,4), 10*kalman_output(:,6))
scatter3(points(1,:), points(2,:), points(3,:))
xlabel('X axis'); ylabel('Y axis'); zlabel('Z axis'); axis equal
