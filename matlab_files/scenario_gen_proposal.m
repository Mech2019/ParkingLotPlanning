clear all; close all; clc;

%% parking lot parameter declaration
discretization = 0.5;

% driving_lane_width = 4.0;
driving_lane_width = 6.0;

slot_width = 2.5;
slot_depth = 5.5;

car_wid = 1.8;
car_len = 4.5;

sensor_rad = 10.0;

total_slot_num = 52;
slot_per_column = [10;8;8;8;8;10];

parking_lot_width = length(slot_per_column)*slot_depth + ...
    length(slot_per_column)/2*driving_lane_width;
parking_lot_length = max(slot_per_column)*slot_width + ...
    2*(driving_lane_width - slot_width);

%% map building
% a zero denotes free space, a one denotes space occupied by parking space
lot = zeros(parking_lot_length/discretization,...
            parking_lot_width/discretization);

% keep track of a parking lot space center grid index
slot_x_index = zeros(total_slot_num,1);
slot_y_index = zeros(total_slot_num,1);

initial_x_offset = ceil(slot_depth/discretization/2);
initial_y_offset = ceil((driving_lane_width - slot_width)/discretization + ...
                        ceil(slot_width/discretization/2));

counter = 1;
for i = 1:length(slot_per_column)
    if (slot_per_column(i) == max(slot_per_column))
        if (i == 1)
            x0 = initial_x_offset;
        else
            x0 = parking_lot_width/discretization - initial_x_offset + 1;
        end
        y0 = initial_y_offset;
    else
        x0 = initial_x_offset + (ceil((i-1)/2) * driving_lane_width + ...
                (i-1)*slot_depth)/discretization;
        y0 = initial_y_offset + slot_width/discretization;
    end
    for j = 1:(slot_per_column(i))
        slot_x_index(counter) = x0;
        slot_y_index(counter) = y0 + (j-1)*slot_width/discretization;
        counter = counter + 1;
    end
end
slot_loc = [slot_x_index, slot_y_index];

% fill the map now
counter = 1;
for i = 1:length(slot_x_index)
    x = slot_loc(i,1); 
    y = slot_loc(i,2);
    for j = x-floor(slot_depth/discretization/2):...
            x+floor(slot_depth/discretization/2)
        for k = y-floor(slot_width/discretization/2):...
                y+floor(slot_width/discretization/2)
%         lot(k,j) = 1;
        lot(k,j) = counter * 5 + 100;
        end
    end
    counter = counter + 1;
end

%% generate numerical map
slot_x = zeros(total_slot_num,1);
slot_y = zeros(total_slot_num,1);

x_offset = slot_depth/2;
y_offset = (driving_lane_width - slot_width) + ...
             (slot_width/2);

counter = 1;
for i = 1:length(slot_per_column)
    if (slot_per_column(i) == max(slot_per_column))
        if (i == 1)
            x0 = x_offset;
        else
            x0 = parking_lot_width - x_offset;
        end
        y0 = y_offset;
    else
        x0 = x_offset + (ceil((i-1)/2) * driving_lane_width + ...
                (i-1)*slot_depth);
        y0 = y_offset + slot_width;
    end
    for j = 1:(slot_per_column(i))
        slot_x(counter) = x0;
        slot_y(counter) = y0 + (j-1)*slot_width;
        counter = counter + 1;
    end
end
slot_xy_flag = [slot_x, slot_y, zeros(size(slot_x)), ones(size(slot_x))];

%% designate empty slots
slot_xy_flag( 2,4) = 0;
slot_xy_flag( 3,4) = 0;
slot_xy_flag( 4,4) = 0;
slot_xy_flag( 5,4) = 0;
slot_xy_flag( 8,4) = 0;
slot_xy_flag( 9,4) = 0;
% slot_xy_flag(14,4) = 0;
slot_xy_flag(18,4) = 0;
slot_xy_flag(21,4) = 0;
slot_xy_flag(23,4) = 0;
slot_xy_flag(24,4) = 0;
slot_xy_flag(31,4) = 0;
% slot_xy_flag(39,4) = 0;   % cheat here
slot_xy_flag(40,4) = 0;
slot_xy_flag(46,4) = 0;   % comment out for map 2

%% data output
csvwrite('../src/map.csv', slot_xy_flag);