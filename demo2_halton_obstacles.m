% demo2_halton_obstacles.m
% Task 2: Place Obstacles, Generate Halton Samples, and Check Collisions
clear; clc; close all;

fprintf('=== Demo 2: Halton Samples with Polyhedron Collision Checking ===\n');

env = warehouse_environment();
num_samples = 1000;
samples = generate_halton_samples_3d(num_samples, env.xRange, env.yRange);

% Extract and combine all obstacles into a single polyshape for fast collision checking
obs_poly = [];
for i = 1:size(env.obstacles, 1)
    ox = env.obstacles(i,1); oy = env.obstacles(i,2);
    ow = env.obstacles(i,3); oh = env.obstacles(i,4);
    obs_poly = [obs_poly; polyshape([ox ox+ow ox+ow ox], [oy oy oy+oh oy+oh])];
end
obs_poly = union(obs_poly);

% Separate colliding vs free samples
free_samples = [];
coll_samples = [];

fprintf('Checking collisions against robot body footprint for %d samples...\n', num_samples);
fprintf('(Note: Since we are projecting robot size, points slightly near obstacles may be filtered)\n');

for i = 1:num_samples
    q = samples(i, :);
    % check_collision_polyhedron evaluates the size of halton_robot_model vs obstacles
    if check_collision_polyhedron(q, obs_poly)
        coll_samples = [coll_samples; q];
    else
        free_samples = [free_samples; q];
    end
end

% Visualization in 2D Space
figure('Name', 'Task 2: Halton Samples vs Obstacles', 'Color', 'w', 'Position', [150, 150, 900, 700]);
hold on; axis equal; box on;
xlim(env.xRange); ylim(env.yRange);
title('2D Projection: Free Space vs Colliding Samples (Robot Size Included)', 'FontSize', 12);
xlabel('X (m)'); ylabel('Y (m)');

% Plot obstacles polyshape
plot(obs_poly, 'FaceColor', [0.4 0.4 0.4], 'EdgeColor', 'k');

% Scatter plot separated points
if ~isempty(free_samples)
    scatter(free_samples(:,1), free_samples(:,2), 20, 'g', 'filled', 'MarkerEdgeColor', 'k', 'DisplayName', 'Free Sample');
end
if ~isempty(coll_samples)
    scatter(coll_samples(:,1), coll_samples(:,2), 20, 'r', 'filled', 'MarkerEdgeColor', 'k', 'DisplayName', 'Colliding Sample');
end

legend('Warehouse Shelves', 'Free (Robot fits)', 'Colliding (Robot crashes)', 'Location', 'best');
