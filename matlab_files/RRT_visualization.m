scenario_gen_proposal;
map = load('../src/map.csv');
traj = load('../src/rrt_sample.csv');

rMAT = @(theta) [cos(theta), -sin(theta); sin(theta), cos(theta)];
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
scatter(traj(:,1),traj(:,2),'.');
% hold off;
hold on;
scatter(30.4784,30.5445,'x','LineWidth',2);
hold off;