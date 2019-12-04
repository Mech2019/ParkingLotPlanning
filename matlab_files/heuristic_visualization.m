scenario_gen_proposal;
map = load('../src/map.csv');

%% 
weight = 100000.0;

x_loc = 0:0.1:parking_lot_width;
y_loc = 0:0.1:parking_lot_length;

shop = [50,30];

heu = @(pt1, pt2) weight * (sum((pt1-pt2).^2) + sum((pt2-shop).^2));

[X,Y] = meshgrid(x_loc, y_loc);

h_map = zeros(size(X));

slot_xy = slot_xy_flag(:,1:2);
%%
for i = 1:size(X,1)
    for j = 1:size(X,2)
        h = realmax;
        for k = 1:length(slot_xy)
            point = [X(i,j), Y(i,j)];
            h = min(h, heu(point, slot_xy(k,:)));
        end
        h_map(i,j) = h;
    end
end

X = flipud(X);
Y = flipud(Y);
h_map = flipud(h_map);

%%
figure()
mesh(X,Y,h_map);
hold on
plot3(X(1,511), Y(1,511), h_map(1,511),'x','LineWidth', 5);
hold off
