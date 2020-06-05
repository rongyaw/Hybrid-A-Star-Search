function boolean = collision_check(obstacles, point)
x = point(1); y = point(2);
boolean = false;
for i = 1:1:length(obstacles(:,1))
    x_min = obstacles(i,1);
    x_max = obstacles(i,2);
    y_min = obstacles(i,3);
    y_max = obstacles(i,4);
    if x > (x_min - 0.2) && x < (x_max + 0.2) && y > (y_min - 0.2) && y < (y_max + 0.2)
        boolean = true;
    end
end
end