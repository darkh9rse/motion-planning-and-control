function visualize_all(env, robot, cspace, skel, path)
% VISUALIZE_ALL  Comprehensive visualization of the morphing robot system.
%
%   visualize_all(env, robot, cspace, skel, path)
%
%   Creates a multi-panel figure showing:
%       1. Warehouse layout with obstacles
%       2. Configuration space (2D)
%       3. Distance transform + Skeleton overlay
%       4. Planned path with robot morphing states
%
%   Any input can be [] to skip that panel.

    fprintf('[visualize_all] Generating visualizations ...\n');
    
    figure('Name', 'Morphing Robot - Warehouse Motion Planning', ...
           'Color', 'w', 'Position', [50 50 1400 900]);
    
    %% --- Panel 1: Warehouse Environment ---
    subplot(2,2,1);
    draw_warehouse(env);
    title('Warehouse Environment', 'FontSize', 13);
    
    %% --- Panel 2: Configuration Space ---
    subplot(2,2,2);
    if ~isempty(cspace)
        draw_cspace(cspace, env);
        title(sprintf('C-Space (robot: %s)', cspace.robotMode), 'FontSize', 13);
    else
        text(0.5, 0.5, 'C-Space not computed', 'HorizontalAlignment', 'center');
        axis off;
    end
    
    %% --- Panel 3: Skeleton / Distance Map ---
    subplot(2,2,3);
    if ~isempty(skel) && ~isempty(cspace)
        draw_skeleton(cspace, skel);
        title('Distance Transform + Skeleton', 'FontSize', 13);
    else
        text(0.5, 0.5, 'Skeleton not computed', 'HorizontalAlignment', 'center');
        axis off;
    end
    
    %% --- Panel 4: Planned Path ---
    subplot(2,2,4);
    if ~isempty(path)
        draw_path(env, path, skel);
        title(sprintf('Planned Path (%.1f m)', path.totalDist), 'FontSize', 13);
    else
        draw_warehouse(env);
        title('Path not computed', 'FontSize', 13);
    end
    
    fprintf('[visualize_all] Done.\n');
end


%% =================== Drawing Functions ===================

function draw_warehouse(env)
% Draw the warehouse floor plan
    hold on;
    
    % Draw floor
    rectangle('Position', [env.xRange(1) env.yRange(1) ...
              env.xRange(2)-env.xRange(1) env.yRange(2)-env.yRange(1)], ...
              'FaceColor', [0.95 0.95 0.95], 'EdgeColor', 'k', 'LineWidth', 2);
    
    % Draw obstacles
    colors = lines(size(env.obstacles, 1));
    for k = 1:size(env.obstacles, 1)
        obs = env.obstacles(k, :);
        if k <= size(env.obstacles,1) - 2
            % Shelf color
            faceCol = [0.6 0.4 0.2];
        else
            % Pillar color
            faceCol = [0.5 0.5 0.5];
        end
        rectangle('Position', obs, 'FaceColor', faceCol, ...
                  'EdgeColor', 'k', 'LineWidth', 1);
    end
    
    % Labels
    xlabel('X (m)'); ylabel('Y (m)');
    axis equal;
    xlim(env.xRange); ylim(env.yRange);
    grid on;
    set(gca, 'Layer', 'top');
    hold off;
end


function draw_cspace(cspace, env)
% Draw the 2D configuration space using rectangles only.
% Each original obstacle is shown as its Minkowski-expanded rectangle.
    hold on;
    
    % Draw floor (free space background)
    rectangle('Position', [env.xRange(1) env.yRange(1) ...
              env.xRange(2)-env.xRange(1) env.yRange(2)-env.yRange(1)], ...
              'FaceColor', [1 1 1], 'EdgeColor', 'k', 'LineWidth', 2);

    % Robot bounding radius used for Minkowski expansion
    r = cspace.resolution * ceil( ...
        max(1, round(0.5 * (size(findobj_se(cspace),1)) )) );
    % Fallback: derive radius from grid difference
    % Instead, compute expansion amount from the robot info stored at creation.
    % We approximate: expansion = robot radius (already baked into grid2D).
    % Recover it: scan the first obstacle to measure how much it grew.
    % Simpler approach: just use bounding radius stored in robotMode dims.
    % Safest: re-derive from the structuring element size used in generate_cspace.
    % Since we have the original obstacles and the grid, just draw both.

    % Get robot radius from the C-space expansion
    % Compare original obstacle grid extent vs C-space obstacle extent
    % For simplicity, estimate from resolution and grid
    % We'll compute it from the first obstacle
    obs1 = env.obstacles(1,:);
    r_est = estimate_expansion(obs1, cspace, env);

    % Draw Minkowski-expanded obstacles as red rectangles
    for k = 1:size(env.obstacles, 1)
        obs = env.obstacles(k, :);
        % Expanded rectangle: grow by r_est on all sides
        ex = obs(1) - r_est;
        ey = obs(2) - r_est;
        ew = obs(3) + 2*r_est;
        eh = obs(4) + 2*r_est;
        % Clip to workspace
        ex = max(env.xRange(1), ex);
        ey = max(env.yRange(1), ey);
        ew = min(env.xRange(2) - ex, ew);
        eh = min(env.yRange(2) - ey, eh);
        rectangle('Position', [ex ey ew eh], ...
                  'FaceColor', [0.9 0.3 0.3 0.4], ...
                  'EdgeColor', [0.7 0.1 0.1], 'LineWidth', 1.5);
    end

    % Draw boundary C-obstacle strips (walls expanded inward)
    % Left wall
    rectangle('Position', [env.xRange(1), env.yRange(1), r_est, env.yRange(2)-env.yRange(1)], ...
              'FaceColor', [0.9 0.3 0.3 0.4], 'EdgeColor', [0.7 0.1 0.1], 'LineWidth', 1);
    % Right wall
    rectangle('Position', [env.xRange(2)-r_est, env.yRange(1), r_est, env.yRange(2)-env.yRange(1)], ...
              'FaceColor', [0.9 0.3 0.3 0.4], 'EdgeColor', [0.7 0.1 0.1], 'LineWidth', 1);
    % Bottom wall
    rectangle('Position', [env.xRange(1), env.yRange(1), env.xRange(2)-env.xRange(1), r_est], ...
              'FaceColor', [0.9 0.3 0.3 0.4], 'EdgeColor', [0.7 0.1 0.1], 'LineWidth', 1);
    % Top wall
    rectangle('Position', [env.xRange(1), env.yRange(2)-r_est, env.xRange(2)-env.xRange(1), r_est], ...
              'FaceColor', [0.9 0.3 0.3 0.4], 'EdgeColor', [0.7 0.1 0.1], 'LineWidth', 1);

    % Overlay original obstacles as blue dashed outlines
    for k = 1:size(env.obstacles, 1)
        obs = env.obstacles(k, :);
        rectangle('Position', obs, 'EdgeColor', 'b', 'LineWidth', 2, ...
                  'LineStyle', '--');
    end
    
    xlabel('X (m)'); ylabel('Y (m)');
    axis equal;
    xlim(env.xRange); ylim(env.yRange);
    grid on;
    set(gca, 'Layer', 'top');
    
    % Legend entries using invisible patches
    patch(nan, nan, [0.9 0.3 0.3], 'FaceAlpha', 0.4, 'EdgeColor', 'none', ...
          'DisplayName', 'C-obstacle (expanded)');
    plot(nan, nan, 'b--', 'LineWidth', 2, 'DisplayName', 'Original obstacle');
    legend('Location', 'northeast', 'FontSize', 8);
    hold off;
end


function r_est = estimate_expansion(obs, cspace, env)
% Estimate the Minkowski expansion radius from the C-space grid.
% Look at how far the C-obstacle extends beyond the original obstacle.
    res = cspace.resolution;
    % Original obstacle right edge in grid coords
    obs_right_col = round((obs(1) + obs(3) - env.xRange(1)) / res) + 1;
    obs_mid_row   = round((obs(2) + obs(4)/2 - env.yRange(1)) / res) + 1;
    
    nx = size(cspace.grid2D, 2);
    obs_mid_row = max(1, min(size(cspace.grid2D,1), obs_mid_row));
    
    % Scan rightward from obstacle edge to find where C-obstacle ends
    expand_cells = 0;
    for c = obs_right_col:nx
        if cspace.grid2D(obs_mid_row, c)
            expand_cells = expand_cells + 1;
        else
            break;
        end
    end
    r_est = expand_cells * res;
    r_est = max(r_est, 0.1);  % minimum fallback
end


function draw_skeleton(cspace, skel)
% Draw distance transform with skeleton overlay
    hold on;
    
    % Distance transform as heatmap
    imagesc(cspace.xVec, cspace.yVec, skel.distMap);
    colormap(gca, parula);
    cb = colorbar;
    ylabel(cb, 'Clearance (m)');
    
    % Overlay skeleton in red
    if ~isempty(skel.skelPoints)
        plot(skel.skelPoints(:,1), skel.skelPoints(:,2), 'r.', 'MarkerSize', 1);
    end
    
    xlabel('X (m)'); ylabel('Y (m)');
    axis equal xy;
    xlim(cspace.xVec([1 end])); ylim(cspace.yVec([1 end]));
    hold off;
end


function draw_path(env, path, skel)
% Draw planned path on warehouse
    hold on;
    draw_warehouse_simple(env);
    
    % Draw skeleton faintly
    if ~isempty(skel) && ~isempty(skel.skelPoints)
        plot(skel.skelPoints(:,1), skel.skelPoints(:,2), '.', ...
             'Color', [0.8 0.8 0.8], 'MarkerSize', 1);
    end
    
    % Draw path
    wp = path.waypoints;
    plot(wp(:,1), wp(:,2), 'b-', 'LineWidth', 2.5);
    
    % Color waypoints by morphing mode
    modeColors = containers.Map();
    modeColors('default') = [0.2 0.6 1.0];
    modeColors('narrow')  = [1.0 0.5 0.2];
    modeColors('flat')    = [0.2 0.9 0.3];
    modeColors('compact') = [0.9 0.2 0.8];
    
    for k = 1:size(wp,1)
        mode_k = path.morphPlan{k};
        if isKey(modeColors, mode_k)
            col = modeColors(mode_k);
        else
            col = [0 0 0];
        end
        plot(wp(k,1), wp(k,2), 'o', 'MarkerSize', 6, ...
             'MarkerFaceColor', col, 'MarkerEdgeColor', 'k');
    end
    
    % Start & Goal markers
    plot(wp(1,1), wp(1,2), 'g^', 'MarkerSize', 14, 'MarkerFaceColor', 'g', 'LineWidth', 2);
    plot(wp(end,1), wp(end,2), 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r', 'LineWidth', 2);
    
    xlabel('X (m)'); ylabel('Y (m)');
    axis equal;
    xlim(env.xRange); ylim(env.yRange);
    grid on;
    
    legend({'Skeleton', 'Path', '', '', 'Start', 'Goal'}, 'Location', 'northeast');
    hold off;
end


function draw_warehouse_simple(env)
% Lightweight warehouse drawing (no fill)
    rectangle('Position', [env.xRange(1) env.yRange(1) ...
              env.xRange(2)-env.xRange(1) env.yRange(2)-env.yRange(1)], ...
              'EdgeColor', 'k', 'LineWidth', 2);
    
    for k = 1:size(env.obstacles, 1)
        obs = env.obstacles(k, :);
        rectangle('Position', obs, 'FaceColor', [0.85 0.7 0.5], ...
                  'EdgeColor', [0.4 0.3 0.1], 'LineWidth', 1);
    end
end
