clear all; clc; close all

data = dlmread('../build_docker/file.txt');

% Initialize the variables
Xform = repmat(eye(4), [1, 1, size(data,1)]);
points = zeros(4, size(data,1));
bearing = zeros(3, size(data,1)); bearing(:,1) = [0 0 1];

running_xform = eye(4);
for i = 1:size(data,1)
   Xform(1:3, 1:4, i) = reshape(data(i,:),[3,4]);
   
%    running_xform = running_xform * Xform(:,:, i); %uncomment for running delta xforms
   running_xform = Xform(:,:, i); % uncomment for logged absolute xform
   
   bearing(:, i) = running_xform(1:3, 1:3) * [0; 0; 1];
   points(:, i) = running_xform * [0; 0; 0; 1];
end


figure
scatter3(points(1,:), points(2,:), points(3,:))
hold on
quiver3(points(1,:), points(2,:), points(3,:), bearing(1,:), bearing(2,:),...
    bearing(3,:))
xlabel('X axis'); ylabel('Y axis'); zlabel('Z axis'); axis equal
