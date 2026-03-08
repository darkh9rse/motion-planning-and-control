function animate_robot(env, path, dt)
% ANIMATE_ROBOT  Animate the morphing robot moving along the planned path.
%
%   animate_robot(env, path)
%   animate_robot(env, path, dt)
%
%   Shows the robot moving through the warehouse, morphing its shape
%   as it encounters different clearance zones.
%
%   Inputs:
%       env   - Warehouse environment struct
%       path  - Path struct from plan_path()
%       dt    - Time step for animation (default 0.05 s)

    if nargin < 3
        dt = 0.05;
    end
    
    fprintf('[animate_robot] Starting animation ...\n');
    
    figure('Name', 'Morphing Robot Animation', 'Color', 'w', ...
           'Position', [100 100 900 700]);
    
    %% --- Draw warehouse ---
    hold on;
    rectangle('Position', [env.xRange(1) env.yRange(1) ...
              env.xRange(2)-env.xRange(1) env.yRange(2)-env.yRange(1)], ...
              'FaceColor', [0.95 0.95 0.95], 'EdgeColor', 'k', 'LineWidth', 2);
    
    for k = 1:size(env.obstacles, 1)
        obs = env.obstacles(k, :);
        rectangle('Position', obs, 'FaceColor', [0.75 0.6 0.4], ...
                  'EdgeColor', 'k', 'LineWidth', 1);
    end
    
    % Draw full path
    wp = path.waypoints;
    plot(wp(:,1), wp(:,2), 'b--', 'LineWidth', 1.5);
    plot(wp(1,1), wp(1,2), 'g^', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    plot(wp(end,1), wp(end,2), 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    
    xlabel('X (m)'); ylabel('Y (m)');
    title('Morphing Robot Navigation', 'FontSize', 14);
    axis equal;
    xlim(env.xRange); ylim(env.yRange);
    grid on;
    
    %% --- Interpolate path for smooth animation ---
    % Remove duplicate waypoints (zero-distance apart) to avoid interp1 error
    segDist = vecnorm(diff(wp), 2, 2);
    keepIdx = [true; segDist > 1e-9];  % always keep first point
    wp = wp(keepIdx, :);
    path.morphPlan = path.morphPlan(keepIdx);
    
    if size(wp,1) < 2
        warning('Path too short to animate.');
        return;
    end
    
    % Create dense waypoints
    totalPts = 200;
    cumDist = [0; cumsum(vecnorm(diff(wp), 2, 2))];
    totalLen = cumDist(end);
    
    sQuery = linspace(0, totalLen, totalPts);
    xInterp = interp1(cumDist, wp(:,1), sQuery, 'linear', 'extrap');
    yInterp = interp1(cumDist, wp(:,2), sQuery, 'linear', 'extrap');
    
    % Interpolate morph mode
    modeAtWp = zeros(size(wp,1), 1);
    modeNames = {'compact', 'narrow', 'flat', 'default'};
    for k = 1:size(wp,1)
        idx = find(strcmp(modeNames, path.morphPlan{k}));
        if ~isempty(idx)
            modeAtWp(k) = idx;
        else
            modeAtWp(k) = 4;
        end
    end
    modeInterp = round(interp1(cumDist, modeAtWp, sQuery, 'nearest'));
    modeInterp(isnan(modeInterp)) = 4;
    
    %% --- Animate ---
    hRobot = fill(0, 0, 'b', 'FaceAlpha', 0.6, 'EdgeColor', 'k', 'LineWidth', 1.5);
    hTrail = plot(NaN, NaN, 'g-', 'LineWidth', 1);
    hText  = text(env.xRange(2)-1, env.yRange(2)-0.5, '', 'FontSize', 11, ...
                  'HorizontalAlignment', 'right', 'FontWeight', 'bold');
    
    trailX = []; trailY = [];
    
    for k = 1:totalPts
        x = xInterp(k);
        y = yInterp(k);
        
        % Get current mode
        mIdx = max(1, min(4, modeInterp(k)));
        modeName = modeNames{mIdx};
        
        % Get robot shape
        rob = morphing_robot(modeName);
        
        % Compute heading from path direction
        if k < totalPts
            theta = atan2(yInterp(k+1) - y, xInterp(k+1) - x);
        else
            theta = atan2(y - yInterp(k-1), x - xInterp(k-1));
        end
        
        % Get footprint
        fp = rob.getFootprint(x, y, theta);
        fp = [fp; fp(1,:)];  % close polygon
        
        % Update robot display
        set(hRobot, 'XData', fp(:,1), 'YData', fp(:,2), ...
            'FaceColor', rob.color);
        
        % Update trail
        trailX = [trailX, x]; %#ok<AGROW>
        trailY = [trailY, y]; %#ok<AGROW>
        set(hTrail, 'XData', trailX, 'YData', trailY);
        
        % Update text
        set(hText, 'String', sprintf('Mode: %s | Pos: (%.1f, %.1f)', modeName, x, y));
        
        drawnow;
        pause(dt);
    end
    
    fprintf('[animate_robot] Animation complete.\n');
    hold off;
end
