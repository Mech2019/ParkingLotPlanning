scenario_gen_proposal;
map = load('../src/map.csv');

%% visualize
% parking lot map
figure()
for i = 1:total_slot_num
    % draw rectangle
    rectangle('Position',[map(i,1) - slot_depth/2, ...
        map(i,2) - slot_width/2, slot_depth, slot_width]);
    hold on;
    if (map(i,4) == 1)
        rectangle('Position',[map(i,1) - car_len/2, ...
            map(i,2) - car_wid/2, car_len, car_wid], ...
            'FaceColor', 'b');
        hold on;
    end
end
% plot ego vehicle for line of sight test!!!
% car position 1
rectangle('Position',[8.5 - car_wid/2, ...
            2.75 - car_len/2, car_wid, car_len], ...
            'FaceColor', 'r');
hold on;
draw_circle(8.5, 2.75, sensor_rad);
hold on;
% % car position 2
% rectangle('Position',[25.5 - car_wid/2, ...
%     10.75 - car_len/2, car_wid, car_len], ...
%     'FaceColor', 'r');
% hold on;
% draw_circle(25.5, 10.75, sensor_rad);
hold off;
axis equal
xlim([0 parking_lot_width]);ylim([0 parking_lot_length]);
% imagesc(lot); axis equal; colorbar; colormap jet; hold on;