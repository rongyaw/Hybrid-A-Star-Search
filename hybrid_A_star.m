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
set(gca,'visible','off')

%%  Create the obstacles and start/goal location
obstacle = [1,2,2,10;
            4,5,0,4;
            -4,-3,-10,-2;
            7,8,2,8;
            -1,0,-5,1;
            -4,-3,0,8;
            -2,-1,0,1;
            3,6,4,5;
            1,3,-1,0;
            1,2,-7,-3;
            4,5,7,10];% Format -> [min_x, max_x, min_y, max_y]
obstacles = [];
% Plot the obstacles in map
for j = 0:15:45
    for i = 1:1:length(obstacle(:,1))
        min_x = obstacle(i,1) + j;
        max_x = obstacle(i,2) + j;
        min_y = obstacle(i,3);
        max_y = obstacle(i,4);
        obstacles = [obstacles; [min_x, max_x, min_y, max_y]];
        obs_x = [min_x, max_x, max_x, min_x, min_x];
        obs_y = [min_y, min_y, max_y, max_y, min_y];
        fill(obs_x, obs_y,'k'); hold on
        clear min_x;
        clear max_x;
        clear min_y;
        clear max_y;
    end
end
% Setup the start and goal location for nvaigation and plot them
start_x = -5;
start_y = -2;
start_yaw = 0;%random('Uniform',0,3.14);
goal_x = 40;
goal_y = 4;
plot(start_x, start_y, 'or', 'MarkerSize', 20, 'MarkerFaceColor', 'r');hold on
plot(goal_x, goal_y, 'or', 'MarkerSize', 20, 'MarkerFaceColor', 'r');hold on
xlim([start_x-2, goal_x+4])
set(gca,'xtick',[])
set(gca,'ytick',[])

%%  Create open_list and closed list
% This is the simplified version a heap structure.
open = []; % Store the information of vertex
open_f = []; % Store the corresponding key values: f(n)
open_g = []; % Store the corresponding key values: g(n)
open_c = []; % Combine the previous two as cost value
close = []; % Create closed list for finding optimal path

%%  Create the steering angle and arc length for sampling
steering = linspace(-0.41,0.41,6);
arc_length = input('Please define sampling distance: ');

%%  Searching over the map to reach the goal from the start
% Initialize the open list
% Global id keep track of the total vertex visited, it serve the function
% as counter
global id
id = 1;
mother_id = 0;
w_gn = input('Please define the weight of g(n): ');
open = [start_x, start_y, start_yaw, 0, mother_id, id, 0];
vertex_sum = [start_x, start_y];
open_f = [open_f, pdist([open(1:2);[goal_x, goal_y]])]; % eucliden heuristic function
open_c = open_f + open(end);
counter = 0;
% Start the timer
tic
while length(open_c) ~= 0
    % Pop the minimum cost value
    [min_cost,source_ind] = min(open_c); % Pop up the smallest key value from open list
    source = open(source_ind,:); % Pop up the smallest key value from open list
    close = [close; [source, min_cost]];
    if pdist([source(1:2);[goal_x, goal_y]]) < 0.5
        break
    end
    open(source_ind,:) = []; % Delete the pop-up key from open list
    open_f(source_ind) = []; % Same as above
    open_c(source_ind) = []; % Same as above
    sample = ackermann_sampler(1, source, steering, arc_length, @collision_check, obstacles, vertex_sum, id); % Search from the pop-up point
    counter = counter + 1;
    disp(['This is the ',num2str(counter),'th sampling!']);
    if ~isempty(sample)
        f = sample(:,1:2) - [goal_x, goal_y];
        f = (f(:,1).^2 + f(:,2).^2).^0.5;
        open = [open; sample];
        vertex_sum = [vertex_sum; sample(:,1:2)];
        open_f = [open_f, f.'];
        open_c = open_f + w_gn*open(:,4).';
        drawnow
        % In this case, the weight of distance travelled is set to be lower than distance to the goal
    end
end
t = toc; % Stop the timer
disp(['Total node explored is ',num2str(length(vertex_sum(:,1)))]);

%%  Search through the closed list to find the path.

%Start from the last point and draw the path between them
[~,min_id] =  max(close(:, 6));
search_id = close(min_id, 5);
plot(close(min_id,1), close(min_id,2),'rs','MarkerSize',6,'MarkerFaceColor','r');hold on
path_point = []; % Store the path from closed list
distance = 0;
while search_id ~= 0
    point_id = find(close(:,6)== search_id);
    path_point = [path_point;[close(point_id,1), close(point_id,2), close(point_id,3)]];
    distance = distance + close(point_id, 7);
    draw_car(close(point_id,1), close(point_id,2), close(point_id,3));
    search_id = close(point_id,5);
end
disp(['Total distance travelled is ', num2str(length(path_point(:,1))*arc_length - arc_length)]);
disp(['Total distance travelled is ', num2str(distance/2.5 + t),' seconds.'])
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
    plot(x_glob, y_glob,'-b','LineWidth',2);hold on
end
plot([path_point(1,1),goal_x],[path_point(1,2),goal_y],'-r','LineWidth',3);hold off
xlim([start_x-2, goal_x+4])