scenario_gen_proposal;
map = load('../src/map.csv');
% traj = load('../src/traj.csv');
traj = load('../src/local_result.csv');
%% static visualize
% % parking lot map
% figure()
% for i = 1:total_slot_num
%     % draw rectangle
%     rectangle('Position',[map(i,1) - slot_depth/2, ...
%         map(i,2) - slot_width/2, slot_depth, slot_width]);
%     hold on;
%     if (map(i,4) == 1)
%         rectangle('Position',[map(i,1) - car_len/2, ...
%             map(i,2) - car_wid/2, car_len, car_wid], ...
%             'FaceColor', 'b');
%         hold on;
%     end
% end
% % plot ego vehicle for line of sight test!!!
% % car position 1
% rectangle('Position',[8.5 - car_wid/2, ...
%             2.75 - car_len/2, car_wid, car_len], ...
%             'FaceColor', 'r');
% hold on;
% draw_circle(8.5, 2.75, sensor_rad);
% % hold on;
% % % car position 2
% % rectangle('Position',[25.5 - car_wid/2, ...
% %     10.75 - car_len/2, car_wid, car_len], ...
% %     'FaceColor', 'r');
% % hold on;
% % draw_circle(25.5, 10.75, sensor_rad);
% hold off;
% axis equal
% xlim([0 parking_lot_width]);ylim([0 parking_lot_length]);
% % imagesc(lot); axis equal; colorbar; colormap jet; hold on;

%% animation
movie_time = 0:0.1:(length(traj)-1)*0.1;
numberOfFrames = length(movie_time);
delay = 0.1;
rMAT = @(theta) [cos(theta), -sin(theta); sin(theta), cos(theta)];
v = VideoWriter('test.mp4', 'MPEG-4');
v.FrameRate = 10;
open(v);
 % plot map
for j = 1:total_slot_num
    % draw rectangle
    rectangle('Position',[map(j,1) - slot_depth/2, ...
        map(j,2) - slot_width/2, slot_depth, slot_width]);
    hold on;
    if (map(j,4) == 1)
        rectangle('Position',[map(j,1) - car_len/2, ...
            map(j,2) - car_wid/2, car_len, car_wid], ...
            'FaceColor', 'b');
        hold on;
    end
end
for i = 1 : numberOfFrames
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
    
%     hold on;
%     c = draw_circle(ego_x, ego_y, sensor_rad);
    hold off;
    axis equal;
    xlim([0 parking_lot_width]);ylim([0 parking_lot_length]);
    
    drawnow;
    
    thisFrame = getframe(gca);
    writeVideo(v, thisFrame);
%     im = frame2im(thisFrame);
%     [imind,cm] = rgb2ind(im,256);
%     if i == 1
%       imwrite(imind,cm,'test.gif','gif', 'DelayTime', delay, 'Loopcount',inf);
%     else
%       imwrite(imind,cm,'test.gif','gif', 'WriteMode','append', 'DelayTime', delay);
%     end
    
    delete(r);
%     delete(c);
end
close(v);