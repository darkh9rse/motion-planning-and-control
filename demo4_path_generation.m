% demo4_path_generation.m
% Task 4: Path Generation & A*/RRT* Optimal Extraction with Robot Base Simulation
clear; clc; close all;

fprintf('=== Demo 4: Path Generation & Simulation ===\n');

env = warehouse_environment();
start_q = [2.0, 1.0, 0];
goal_q = [18.0, 13.5, pi/2];
max_nodes = 2000;

fprintf('Running RRT* to find optimal global path linking Start to Goal...\n');
[path, ~] = build_halton_rrt_star(start_q, goal_q, env, max_nodes);

if isempty(path)
    disp('Path not found within specified max_nodes.');
    return;
end

% Visualization
figure('Name', 'Task 4: Final Path Validation', 'Color', 'w', 'Position', [250, 250, 1000, 800]);
ax = axes; hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on'); box(ax, 'on');
xlim(ax, env.xRange); ylim(ax, env.yRange);
title('Final Generated Trajectory and Navigation Sandbox', 'FontSize', 14);

% Plot obstacles
for i = 1:size(env.obstacles, 1)
    ox = env.obstacles(i,1); oy = env.obstacles(i,2);
    ow = env.obstacles(i,3); oh = env.obstacles(i,4);
    rectangle('Position', [ox, oy, ow, oh], 'FaceColor', [0.4 0.4 0.4]);
end

% Plot Extracted Path Line
plot(ax, path(:,1), path(:,2), 'k-', 'LineWidth', 3.0);
plot(ax, start_q(1), start_q(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(ax, goal_q(1), goal_q(2), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm');

disp('Displaying a brief mid-route snapshot of the 5D capable robot model traversing (d1=0, d2=0)...');
q_mid = path(round(size(path,1)/2), :);
% Ensure size extends tracking dimensions
q_mid = [q_mid(1), q_mid(2), q_mid(3), 0, 0]; 

poly_rob = halton_robot_model(q_mid);
plot(ax, poly_rob, 'FaceColor', [0.2 1.0 0.4], 'FaceAlpha', 0.8, 'EdgeColor', 'k');

text(ax, q_mid(1), q_mid(2)+1.5, 'Robot Footprint Snapshot', 'HorizontalAlignment', 'center', 'Color', [0 0.5 0], 'FontWeight', 'bold');
