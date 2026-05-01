%% main_halton_planning.m
% Master Script outlining the full pipeline for:
% 1. Global 3D Planning (x, y, theta) using Halton RRT*
% 2. Polyhedron (polyshape) collision detection
% 3. Local actions showing d1 (actuator) and d2 (fingers)

clear; clc; close all;

fprintf('============================================\n');
fprintf('  3D Halton-based RRT* Motion Planning      \n');
fprintf('============================================\n\n');

%% 1. Setup Environment
% Reusing the layout and obstacle bounds from the built-in warehouse environment.
env = warehouse_environment();

% Define Start and Goal states globally around the warehouse
start_q = [2.0, 1.0, 0];
goal_q = [18.0, 13.5, pi/2];

%% 2. Run RRT* with Halton Sampling
fprintf('Initializing RRT* Planner in 3D C-Space (x, y, theta)...\n');
max_nodes = 2000; % Increased nodes to allow complex navigation
[path, tree] = build_halton_rrt_star(start_q, goal_q, env, max_nodes);

if isempty(path)
    disp('Failed to find path. Try increasing max_nodes or reducing step limits.');
    return;
end

%% 3. Visualization
fprintf('Visualization of Graph/Tree and Obstacles...\n');
fig = figure('Name', 'Halton RRT* Path Planning', 'Color', 'w', 'Position', [100 50 1000 800]);
ax = axes('Parent', fig);
hold(ax, 'on'); axis(ax, 'equal');
xlim(ax, env.xRange); ylim(ax, env.yRange);
grid(ax, 'on'); box(ax, 'on');

% Plot obstacles
for i = 1:size(env.obstacles, 1)
    ox = env.obstacles(i,1); oy = env.obstacles(i,2);
    ow = env.obstacles(i,3); oh = env.obstacles(i,4);
    rectangle(ax, 'Position', [ox, oy, ow, oh], 'FaceColor', [0.4 0.4 0.4]);
end

% Plot RRT* Tree
for i = 2:size(tree.nodes, 1)
    p_idx = tree.parents(i);
    if p_idx > 0
        plot(ax, [tree.nodes(i,1), tree.nodes(p_idx,1)], ...
                 [tree.nodes(i,2), tree.nodes(p_idx,2)], 'Color', [0.8 0.9 1.0]);
    end
end

% Plot Final Valid Path
plot(ax, path(:,1), path(:,2), 'k-', 'LineWidth', 2.5);
plot(ax, start_q(1), start_q(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
text(ax, start_q(1), start_q(2)-0.5, 'START', 'Color', 'g', 'FontWeight', 'bold');

plot(ax, goal_q(1), goal_q(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
text(ax, goal_q(1), goal_q(2)+0.5, 'GOAL', 'Color', 'r', 'FontWeight', 'bold');

%% 4. Animation of Robot Navigation
fprintf('Animating global traversal (d1 and d2 inactive)...\n');
robot_patch = [];

% Interpolate path for smoother animation
smooth_path = [];
for i = 1:size(path,1)-1
    dist_val = norm(path(i+1, 1:2) - path(i, 1:2));
    steps = max(2, ceil(dist_val / 0.1));
    xs = linspace(path(i,1), path(i+1,1), steps);
    ys = linspace(path(i,2), path(i+1,2), steps);
    
    % Handle angle wrap
    th_start = path(i,3);
    th_end = path(i+1,3);
    th_diff = mod((th_end - th_start) + pi, 2*pi) - pi;
    ths = th_start + linspace(0, 1, steps) * th_diff;
    
    interp_chunk = [xs', ys', ths'];
    if i == 1
        smooth_path = interp_chunk;
    else
        smooth_path = [smooth_path; interp_chunk(2:end, :)]; %#ok<AGROW>
    end
end

for i = 1:size(smooth_path, 1)
    % During navigation, actuator (d1) and fingers (d2) are 0
    q = [smooth_path(i, 1), smooth_path(i, 2), smooth_path(i, 3), 0, 0];
    poly_rob = halton_robot_model(q);
    
    if ~isempty(robot_patch)
        delete(robot_patch);
    end
    robot_patch = plot(ax, poly_rob, 'FaceColor', [0.2 0.6 1.0], 'FaceAlpha', 0.8, 'EdgeColor', 'k');
    drawnow;
end

%% 5. Local Pick/Place Morphing Action
fprintf('Simulating local joint execution (d1, d2) near target...\n');

obj_x = goal_q(1) + 2.0*cos(goal_q(3));
obj_y = goal_q(2) + 2.0*sin(goal_q(3));
plot(ax, obj_x, obj_y, 's', 'MarkerSize', 14, 'MarkerFaceColor', [0.85 0.55 0.25], 'MarkerEdgeColor', 'k');
text(ax, obj_x, obj_y+0.6, 'CARGO', 'HorizontalAlignment', 'center');

% Actuator Extends (d1: 0 -> 1)
for d1 = linspace(0, 1, 15)
    q = [goal_q(1), goal_q(2), goal_q(3), d1, 0];
    poly_rob = halton_robot_model(q);
    delete(robot_patch);
    robot_patch = plot(ax, poly_rob, 'FaceColor', [0.2 0.6 1.0], 'FaceAlpha', 0.8);
    drawnow; 
end

% Fingers Open (d2: 0 -> 1)
for d2 = linspace(0, 1, 15)
    q = [goal_q(1), goal_q(2), goal_q(3), 1, d2];
    poly_rob = halton_robot_model(q);
    delete(robot_patch);
    robot_patch = plot(ax, poly_rob, 'FaceColor', [0.2 0.6 1.0], 'FaceAlpha', 0.8);
    drawnow;
end

% Wait for cargo pick
pause(0.5);

% Fingers Close around cargo (d2: 1 -> 0)
for d2 = linspace(1, 0, 15)
    q = [goal_q(1), goal_q(2), goal_q(3), 1, d2];
    poly_rob = halton_robot_model(q);
    delete(robot_patch);
    robot_patch = plot(ax, poly_rob, 'FaceColor', [0.2 0.6 1.0], 'FaceAlpha', 0.8);
    drawnow;
end

% Actuator Retracts pulling cargo (d1: 1 -> 0)
for d1 = linspace(1, 0, 15)
    q = [goal_q(1), goal_q(2), goal_q(3), d1, 0];
    poly_rob = halton_robot_model(q);
    delete(robot_patch);
    robot_patch = plot(ax, poly_rob, 'FaceColor', [0.2 0.6 1.0], 'FaceAlpha', 0.8);
    drawnow;
end

fprintf('Pipeline completed.\n');
