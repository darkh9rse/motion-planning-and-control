%% MAIN_PIPELINE.m
%  Master script: Morphing Robot for Warehouse Operations
%  -------------------------------------------------------
%  This script connects all modules together:
%    1. Create warehouse environment
%    2. Define morphing robot (multiple modes)
%    3. Generate configuration space (C-space)
%    4. Extract skeleton / Voronoi roadmap
%    5. Plan a path from start to goal
%    6. Visualize everything
%    7. Animate the robot
%    8. (Optional) Compare all morphing modes
%
%  Authors : [Your Names]
%  Course  : Motion Planning and Control
%  Date    : March 2026
%  -------------------------------------------------------

clear; clc; close all;
fprintf('============================================\n');
fprintf('  Morphing Robot for Warehouse Operations   \n');
fprintf('============================================\n\n');

%% ========== STEP 1: Create Warehouse Environment ==========
fprintf('--- Step 1: Building warehouse environment ---\n');
env = warehouse_environment();
fprintf('\n');

%% ========== STEP 2: Define the Morphing Robot ==========
fprintf('--- Step 2: Defining morphing robot ---\n');
% Start with default mode; the planner will suggest morphing
robot = morphing_robot('default');

% Also show all modes
fprintf('\nAll available modes:\n');
allModes = robot.allModes;
for k = 1:length(allModes)
    r = morphing_robot(allModes{k});
end
fprintf('\n');

%% ========== STEP 3: Generate Configuration Space ==========
fprintf('--- Step 3: Generating Configuration Space ---\n');
fprintf('  (This may take a minute for the 3D C-space...)\n');

% Use fewer theta slices for faster computation (12 => 30 deg steps)
nTheta = 12;
cspace = generate_cspace(env, robot, nTheta);
fprintf('\n');

%% ========== STEP 4: Generate Skeleton Roadmap ==========
fprintf('--- Step 4: Computing skeleton of free space ---\n');
fprintf('  (Thinning algorithm - may take some time on fine grids...)\n');

% NOTE: If the grid is very fine (0.05m resolution), thinning is slow.
%       For a quick test, you can temporarily set env.resolution = 0.1
%       in warehouse_environment.m
skel = generate_skeleton(cspace);
fprintf('\n');

%% ========== STEP 5: Plan a Path ==========
fprintf('--- Step 5: Planning path ---\n');

% Define start and goal positions
startPos = [1.0, 0.5];       % bottom-left area
goalPos  = [18.0, 13.5];     % top-right area

path = plan_path(skel, startPos, goalPos, robot);
fprintf('\n');

%% ========== STEP 6: Visualize Everything ==========
fprintf('--- Step 6: Visualization ---\n');
visualize_all(env, robot, cspace, skel, path);
fprintf('\n');

%% ========== STEP 7: Animate the Robot ==========
fprintf('--- Step 7: Animation ---\n');
fprintf('  (Close the animation window to continue)\n');
animate_robot(env, path, 0.03);
fprintf('\n');

%% ========== STEP 8: Morphing Mode Comparison ==========
fprintf('--- Step 8: Morphing Mode Analysis ---\n');
morph_analysis = analyze_morphing(env);
fprintf('\n');

%% ========== Summary ==========
fprintf('============================================\n');
fprintf('  Pipeline Complete!\n');
fprintf('  Path length  : %.2f m\n', path.totalDist);
fprintf('  Waypoints    : %d\n', size(path.waypoints, 1));
fprintf('  Robot modes  : %d available\n', length(allModes));
fprintf('============================================\n');
