% demo3_rrt_tree_generation.m
% Task 3: Show RRT* Tree Expansion
clear; clc; close all;

fprintf('=== Demo 3: RRT* Tree Generation ===\n');

env = warehouse_environment();
start_q = [2.0, 1.0, 0];
goal_q = [18.0, 13.5, pi/2];
max_nodes = 1200; % Reduced slightly to focus purely on tree visual shape

fprintf('Proceeding to calculate RRT* graph out of Halton sequence...\n');
[~, tree] = build_halton_rrt_star(start_q, goal_q, env, max_nodes);

% Visualization
figure('Name', 'Task 3: Halton RRT* Tree', 'Color', 'w', 'Position', [200, 200, 1000, 800]);
ax = axes; hold(ax, 'on'); axis(ax, 'equal'); box(ax, 'on'); grid(ax, 'on');
xlim(ax, env.xRange); ylim(ax, env.yRange);
title('RRT* Tree Expanded through Free Space', 'FontSize', 14);

% Plot obstacles
for i = 1:size(env.obstacles, 1)
    ox = env.obstacles(i,1); oy = env.obstacles(i,2);
    ow = env.obstacles(i,3); oh = env.obstacles(i,4);
    rectangle('Position', [ox, oy, ow, oh], 'FaceColor', [0.3 0.3 0.3]);
end

% Plot Tree Edges
disp('Plotting nodes and branches...');
for i = 2:size(tree.nodes, 1)
    p_idx = tree.parents(i);
    if p_idx > 0
        plot(ax, [tree.nodes(i,1), tree.nodes(p_idx,1)], ...
                 [tree.nodes(i,2), tree.nodes(p_idx,2)], 'Color', [0.2 0.7 0.9], 'LineWidth', 0.5);
    end
end

% Plot Tree Nodes (scatter over edges)
scatter(ax, tree.nodes(:,1), tree.nodes(:,2), 6, 'k', 'filled');

plot(ax, start_q(1), start_q(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(ax, goal_q(1), goal_q(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

text(ax, start_q(1), start_q(2)-0.5, 'Start', 'FontWeight', 'bold');
text(ax, goal_q(1), goal_q(2)+0.5, 'Target', 'FontWeight', 'bold');
