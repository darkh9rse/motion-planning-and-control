function morph_analysis = analyze_morphing(env, robot_modes)
% ANALYZE_MORPHING  Compare C-spaces and reachability across robot morphing modes.
%
%   morph_analysis = analyze_morphing(env, robot_modes)
%
%   For each morphing mode, computes the C-space and measures:
%       - Free space percentage
%       - Number of connected components
%       - Maximum clearance achievable
%
%   Inputs:
%       env          - Warehouse environment struct
%       robot_modes  - Cell array of mode strings (default: all modes)
%
%   Returns:
%       morph_analysis.modes         - Cell array of modes tested
%       morph_analysis.freePercent   - Free space % for each mode
%       morph_analysis.maxClearance  - Max clearance for each mode
%       morph_analysis.cspaces       - Cell array of C-space structs

    if nargin < 2
        robot_modes = {'default', 'narrow', 'flat', 'compact'};
    end
    
    nModes = length(robot_modes);
    
    morph_analysis.modes = robot_modes;
    morph_analysis.freePercent  = zeros(nModes, 1);
    morph_analysis.maxClearance = zeros(nModes, 1);
    morph_analysis.cspaces = cell(nModes, 1);
    
    fprintf('=== Morphing Analysis ===\n');
    fprintf('%-10s | %-8s | %-8s | %-12s | %-12s\n', ...
        'Mode', 'Width', 'Height', 'Free Space %', 'Max Clearance');
    fprintf('%s\n', repmat('-', 1, 60));
    
    for k = 1:nModes
        mode = robot_modes{k};
        rob = morphing_robot(mode);
        
        % Generate C-space (2D only for speed - skip 3D)
        csp = generate_cspace_2D_only(env, rob);
        
        morph_analysis.cspaces{k}     = csp;
        morph_analysis.freePercent(k)  = 100 * sum(csp.freeSpace2D(:)) / numel(csp.freeSpace2D);
        morph_analysis.maxClearance(k) = max(csp.distMap(:));
        
        fprintf('%-10s | %-8.2f | %-8.2f | %10.1f %% | %10.2f m\n', ...
            mode, rob.width, rob.height, ...
            morph_analysis.freePercent(k), morph_analysis.maxClearance(k));
    end
    
    %% --- Visualization: Compare C-spaces ---
    figure('Name', 'Morphing Mode Comparison', 'Color', 'w', ...
           'Position', [100 100 1200 400]);
    
    for k = 1:nModes
        subplot(1, nModes, k);
        csp = morph_analysis.cspaces{k};
        imagesc(csp.xVec, csp.yVec, csp.grid2D);
        colormap(gca, [1 1 1; 0.8 0.2 0.2]);
        axis equal xy;
        xlim([csp.xVec(1) csp.xVec(end)]);
        ylim([csp.yVec(1) csp.yVec(end)]);
        title(sprintf('%s (%.0f%% free)', robot_modes{k}, morph_analysis.freePercent(k)), ...
              'FontSize', 11);
        xlabel('X (m)'); ylabel('Y (m)');
    end
    
    sgtitle('C-Space Comparison Across Morphing Modes', 'FontSize', 14, 'FontWeight', 'bold');
    
    %% --- Bar Chart ---
    figure('Name', 'Morphing Mode Stats', 'Color', 'w', 'Position', [100 550 600 350]);
    bar(morph_analysis.freePercent);
    set(gca, 'XTickLabel', robot_modes);
    ylabel('Free Space (%)');
    title('Free Space vs Robot Morphing Mode');
    grid on;
    
    fprintf('=========================\n');
end


function csp = generate_cspace_2D_only(env, robot)
% Fast 2D-only C-space generation (skip 3D for comparison)
    res = env.resolution;
    xVec = env.xRange(1):res:env.xRange(2);
    yVec = env.yRange(1):res:env.yRange(2);
    
    r_cells = ceil(robot.radius / res);
    [dx, dy] = meshgrid(-r_cells:r_cells, -r_cells:r_cells);
    se = (dx.^2 + dy.^2) <= r_cells^2;
    
    grid2D = imdilate_local(env.grid, se);
    
    grid2D(1:r_cells, :) = true;
    grid2D(end-r_cells+1:end, :) = true;
    grid2D(:, 1:r_cells) = true;
    grid2D(:, end-r_cells+1:end) = true;
    
    % Distance transform
    distMap = bwdist_local(~grid2D) * res;
    
    csp.grid2D = grid2D;
    csp.freeSpace2D = ~grid2D;
    csp.xVec = xVec;
    csp.yVec = yVec;
    csp.resolution = res;
    csp.distMap = distMap;
end


function dilated = imdilate_local(grid, se)
    [m, n] = size(grid);
    [sm, sn] = size(se);
    padm = floor(sm/2);
    padn = floor(sn/2);
    padded = false(m + 2*padm, n + 2*padn);
    padded(padm+1:padm+m, padn+1:padn+n) = grid;
    dilated = false(m, n);
    for i = 1:m
        for j = 1:n
            region = padded(i:i+sm-1, j:j+sn-1);
            if any(region(se))
                dilated(i,j) = true;
            end
        end
    end
end


function dist = bwdist_local(binaryMask)
    [m, n] = size(binaryMask);
    dist = inf(m, n);
    dist(binaryMask) = 0;
    for i = 1:m
        for j = 1:n
            if dist(i,j) > 0
                val = dist(i,j);
                if i > 1, val = min(val, dist(i-1,j)+1); end
                if j > 1, val = min(val, dist(i,j-1)+1); end
                if i > 1 && j > 1, val = min(val, dist(i-1,j-1)+1.414); end
                if i > 1 && j < n, val = min(val, dist(i-1,j+1)+1.414); end
                dist(i,j) = val;
            end
        end
    end
    for i = m:-1:1
        for j = n:-1:1
            if dist(i,j) > 0
                val = dist(i,j);
                if i < m, val = min(val, dist(i+1,j)+1); end
                if j < n, val = min(val, dist(i,j+1)+1); end
                if i < m && j < n, val = min(val, dist(i+1,j+1)+1.414); end
                if i < m && j > 1, val = min(val, dist(i+1,j-1)+1.414); end
                dist(i,j) = val;
            end
        end
    end
end
