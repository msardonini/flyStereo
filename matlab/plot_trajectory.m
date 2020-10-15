clear all; clc; close all
clear all; clc;

data = dlmread('../build_docker/file.txt');

% data = dlmread('/home/msardonini/mntNano/fly_stereo/build/file.txt');
% data = dlmread('/home/msardonini/git/fly_stereo/replay_data/flight/run011/trajecotry.txt');


pose = data(:,1:12);
kalman_output = data(:,13:end);
% Initialize the variables
Xform = repmat(eye(4), [1, 1, size(data,1)]);
points = zeros(4, size(data,1));
velocity = zeros(4, size(data,1));
bearing = zeros(3, size(data,1)); bearing(:,1) = [0 0 1];
euler = zeros(3, size(data,1));

running_xform = eye(4);
for i = 1:size(data,1)
   Xform(1:3, 1:4, i) = reshape(pose(i,:),[3,4]);

   
%    running_xform = running_xform * Xform(:,:, i); %uncomment for running delta xforms
   running_xform = Xform(:,:, i); % uncomment for logged absolute xform
   
   euler(:,i) = R2Euler(running_xform);
   bearing(:, i) = running_xform(1:3, 1:3) * [0; 0; 1];
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
plot(euler(1,:))
hold on
plot(euler(2,:))
plot(euler(3,:))


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
 
