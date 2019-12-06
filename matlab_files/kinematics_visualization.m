clear all; close all; clc;

%% this script checks for kinematics update
% x, y, theta, v, delta
% start = [42.5, 12.9548, 1.85903, 5, 0];
start = [42.5, 12.9548, pi*3/2, 5, 0];

delta_max = deg2rad(30);
delta_min = deg2rad(-30);

v_max = 3;
v_min = -3;

rand_v = v_min + (v_max - v_min) * rand;
rand_delta = delta_min + (delta_max - delta_min) * rand;

duration = 0.5;
sample = 10;
dt = duration/sample;
L = 2.7;

traj = start;
for i = 1:sample
    del = rand_delta;
    dtheta = traj(i,4)/L*del;
    x = traj(i,1) + traj(i,4) * cos(traj(i,3)) * dt;
    y = traj(i,2) + traj(i,4) * sin(traj(i,3)) * dt;
    theta = traj(i,3) + dtheta*dt;
    traj(i+1,:) = [x,y,theta,traj(i,4),del];
end

%% visualize
car_wid = 1.8;
car_len = 4.5;
rMAT = @(theta) [cos(theta), -sin(theta); sin(theta), cos(theta)];
for i = 1 : length(traj)
    % plot ego vehicle
    ego_x = traj(i, 1);
    ego_y = traj(i, 2);
    ego_theta = traj(i, 3);
    % lower left point
    point_ll = [- car_len/2; -car_wid/2];
    % lower right point
    point_lr = [car_len/2; -car_wid/2];
    % top right point
    point_tr = [car_len/2; car_wid/2];
    % top left point
    point_tl = [-car_len/2; car_wid/2];
    % normal car position
    point_car = [point_ll, point_lr, point_tr, point_tl];
    % rotated car position
    point_rotated = rMAT(ego_theta)*point_car;
    % translate back to position
    point_translated = point_rotated + [ego_x;ego_y]*ones(1,length(point_car));
    % plot rectangle
    r = patch('Vertices',point_translated', 'Faces',[1 2 3 4], ...
        'Facecolor','r');

    hold on;
end
hold off;
axis equal;