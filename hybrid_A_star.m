%%  Implementation of hybrid a star for demo purpose
%{
-Author: Rongyao Wang 
-Institution: Clemson University
-Reference: Dmitiri Dolgov, Sebastian Thrun, Micheal Montemerlo and James
Diebel. Path Planning for Autonomous Vehicles in Unknown Semi-structured
Environments

This work is the demonstration of hybrid A* search based on standford AI &
Robotic Group's work in DARPA Grand Challenges
%}
clear all
clc
close all
dbstop if error
%%  Create the obstacles and start/goal location
obstacles = [1,2,0,10;
            4,5,-7,4;
            -5,-3,-10,-2;
            7,8,2,8;
            -1,0,-2,3];% Format -> [min_x, max_x, min_y, max_y]

for i = 1:1:length(obstacles(:,1))
    min_x = obstacles(i,1);
    max_x = obstacles(i,2);
    min_y = obstacles(i,3);
    max_y = obstacles(i,4);
    obs_x = [min_x, max_x, max_x, min_x, min_x];
    obs_y = [min_y, min_y, max_y, max_y, min_y];
    plot(obs_x, obs_y, 'k-', 'Linewidth',5);hold on;
    clear min_x; 
    clear max_x; 
    clear min_y; 
    clear max_y;
end

% Setup the start and goal location for nvaigation
start_x = -7;
start_y = -6;
start_yaw = 0;
goal_x = 10;
goal_y = 2;
plot(start_x, start_y, '>b', 'MarkerSize', 15, 'MarkerFaceColor', 'b');hold on
plot(goal_x, goal_y, '>r', 'MarkerSize', 15, 'MarkerFaceColor', 'r');hold on
%%  Create open_list and closed list
open = [];
open_f = [];
open_g = [];
open_c = [];
close = [];
%%  Create the steering angle and arc length for sampling
steering = linspace(-0.4,0.4,8);
arc_length = 1;
%%  Searching over the map to reach the goal from the start
% Initialize the open list
% Global id keep track of the total vertex visited
global id
id = 1;
mother_id = 0;
open = [start_x, start_y, start_yaw, 0, mother_id, id];
open_f = [open_f, pdist([open(1:2);[goal_x, goal_y]])]; % eucliden heuristic function
open_c = open_f + open(end);
while length(open_c) ~= 0
    % Pop the minimum cost value
    [~,source_ind] = min(open_c);
    source = open(source_ind,:); % Pop up the smallest key value from open list
    close = [close; source];
    if pdist([source(1:2);[goal_x, goal_y]]) < 0.5
        break
    end
    open(source_ind,:) = []; % Delete the pop-up key from open list
    open_f(source_ind) = []; % Same as above
    open_c(source_ind) = []; % Same as above
    sample = ackermann_sampler(source, steering, arc_length, @collision_check, obstacles, id); % Search from the pop-up point
    if length(sample) > 0
        f = sample(:,1:2) - [goal_x, goal_y];
        f = (f(:,1).^2 + f(:,2).^2).^0.5;
        open = [open; sample];
        open_f = [open_f, f.'];
        open_c = open_f + 0.3*open(:,4).';
        % In this case, the weight of distance travelled is set to be lower than distance to the goal
    end
end
%%  Search through the closed list to find the path.
% Start from the last point and draw the path between them
search_id =  close(end, 5);
path_point = [];
while search_id ~= 0
    point_id = find(close(:,6)== search_id);
    path_point = [path_point;[close(point_id,1), close(point_id,2), close(point_id,3)]];
    plot(close(point_id,1), close(point_id,2),'rs','MarkerSize',6,'MarkerFaceColor','r');hold on
    search_id = close(point_id,5);
end

%% Draw the path line
for i = length(path_point):-1:2
    alpha = path_point(i-1,3) - path_point(i,3);
    turning_radius = arc_length/alpha;
    arc = linspace(0.01, arc_length, 8);
    data_x = zeros(1,length(arc_length));
    data_y = zeros(1,length(arc_length));
    for j = 1:1:length(arc)
        alpha = arc(j)/turning_radius;
        data_x(j) = turning_radius*sin(alpha);
        data_y(j) = turning_radius*(1-cos(alpha));
    end
    x_glob = path_point(i,1) + data_x*cos(path_point(i,3)) - data_y*sin(path_point(i,3));
    y_glob = path_point(i,2) + data_x*sin(path_point(i,3)) + data_y*cos(path_point(i,3));
    plot(x_glob, y_glob,'-r','LineWidth',3);hold on
end
plot([path_point(1,1),goal_x],[path_point(1,2),goal_y],'-r','LineWidth',3);hold off
title('Hybrid A* Search Demo','FontSize',15);
xlim([-8,12]);grid on