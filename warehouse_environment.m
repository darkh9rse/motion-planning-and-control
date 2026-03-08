function env = warehouse_environment()
% WAREHOUSE_ENVIRONMENT  Define the warehouse layout with obstacles (shelves, walls).
%
%   env = warehouse_environment()
%
%   Returns a struct with:
%       env.xRange      - [xmin xmax] workspace x-limits (meters)
%       env.yRange      - [ymin ymax] workspace y-limits (meters)
%       env.resolution  - grid resolution (meters per cell)
%       env.obstacles   - Nx4 matrix, each row = [x y width height] of a rectangular obstacle
%       env.grid        - Binary occupancy grid (1=obstacle, 0=free)
%       env.shelves     - Same as obstacles (semantic alias)
%       env.aisleWidth  - Width of aisles between shelves (meters)

    %% --- Warehouse Dimensions ---
    env.xRange = [0 20];       % 20m x 15m warehouse
    env.yRange = [0 15];
    env.resolution = 0.05;     % 5 cm grid resolution

    %% --- Define Shelves as Rectangular Obstacles ---
    % Each row: [x_bottomleft, y_bottomleft, width, height]
    % Shelves are arranged in rows with aisles between them
    env.aisleWidth = 2.0;  % 2m wide aisles
    
    shelfWidth  = 1.0;   % each shelf is 1m wide
    shelfLength = 5.0;   % each shelf is 5m long
    
    obstacles = [];
    
    % --- Row 1 of shelves (y ~ 2) ---
    obstacles = [obstacles; 2,  2, shelfLength, shelfWidth];
    obstacles = [obstacles; 9,  2, shelfLength, shelfWidth];
    obstacles = [obstacles; 16, 2, 3.0,         shelfWidth];
    
    % --- Row 2 of shelves (y ~ 5) ---
    obstacles = [obstacles; 2,  5, shelfLength, shelfWidth];
    obstacles = [obstacles; 9,  5, shelfLength, shelfWidth];
    obstacles = [obstacles; 16, 5, 3.0,         shelfWidth];
    
    % --- Row 3 of shelves (y ~ 8) ---
    obstacles = [obstacles; 2,  8, shelfLength, shelfWidth];
    obstacles = [obstacles; 9,  8, shelfLength, shelfWidth];
    obstacles = [obstacles; 16, 8, 3.0,         shelfWidth];
    
    % --- Row 4 of shelves (y ~ 11) ---
    obstacles = [obstacles; 2,  11, shelfLength, shelfWidth];
    obstacles = [obstacles; 9,  11, shelfLength, shelfWidth];
    obstacles = [obstacles; 16, 11, 3.0,         shelfWidth];
    
    % --- Some scattered smaller obstacles (pillars, stations) ---
    obstacles = [obstacles; 0.5, 7,  0.4, 0.4];   % pillar
    obstacles = [obstacles; 19,  7,  0.4, 0.4];   % pillar
    
    env.obstacles = obstacles;
    env.shelves   = obstacles(1:end-2, :);  % shelves only (without pillars)
    
    %% --- Build Binary Occupancy Grid ---
    nx = round((env.xRange(2) - env.xRange(1)) / env.resolution);
    ny = round((env.yRange(2) - env.yRange(1)) / env.resolution);
    env.grid = false(ny, nx);
    
    for k = 1:size(obstacles,1)
        ox = obstacles(k,1);
        oy = obstacles(k,2);
        ow = obstacles(k,3);
        oh = obstacles(k,4);
        
        % Convert to grid indices
        c1 = max(1, floor((ox - env.xRange(1)) / env.resolution) + 1);
        c2 = min(nx, floor((ox + ow - env.xRange(1)) / env.resolution) + 1);
        r1 = max(1, floor((oy - env.yRange(1)) / env.resolution) + 1);
        r2 = min(ny, floor((oy + oh - env.yRange(1)) / env.resolution) + 1);
        
        env.grid(r1:r2, c1:c2) = true;
    end
    
    fprintf('[warehouse_environment] Created %dm x %dm warehouse with %d obstacles.\n', ...
        env.xRange(2)-env.xRange(1), env.yRange(2)-env.yRange(1), size(obstacles,1));
end
