function draw_car(x,y,yaw)
% draw the car in car coordinate
wheelbase = 0.5;
trackwidth = 0.3;
xf = wheelbase*0.4;
xr = - wheelbase*0.6;
yl = trackwidth/2;
yr = - trackwidth/2;

p0 = [xf, yl]; p1 = [xf, yr];
p2 = [xr, yr]; p3 = [xr, yl];

% setup coordinate transformation matrix
coord_trans = [cos(yaw), -sin(yaw);sin(yaw), cos(yaw)];
p00 = [x;y] + coord_trans * p0.';
p11 = [x;y] + coord_trans * p1.';
p22 = [x;y] + coord_trans * p2.';
p33 = [x;y] + coord_trans * p3.';

pos_x = [p00(1), p11(1), p22(1), p33(1), p00(1)];
pos_y = [p00(2), p11(2), p22(2), p33(2), p00(2)];
fill(pos_x, pos_y,'y');hold on
end