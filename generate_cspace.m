function cspace = generate_cspace(env, robot, nTheta)
% GENERATE_CSPACE  Build the configuration space for the morphing robot.
%
%   cspace = generate_cspace(env, robot)
%   cspace = generate_cspace(env, robot, nTheta)
%
%   For a 2D workspace with orientation, the C-space is (x, y, theta).
%   We discretize theta into nTheta slices (default 36 => 10 deg steps).
%
%   For each (x_i, y_j, theta_k), we check if the robot footprint collides
%   with the obstacle grid. The result is a 3D binary C-space volume.
%
%   For the simplified 2D case (theta=0), we use Minkowski sum expansion
%   of obstacles by the robot's bounding shape.
%
%   Returns:
%       cspace.grid3D       - 3D logical array [ny x nx x nTheta] (true=collision)
%       cspace.grid2D       - 2D logical array [ny x nx] (Minkowski expansion, theta=0)
%       cspace.xVec         - x coordinate vector
%       cspace.yVec         - y coordinate vector
%       cspace.thetaVec     - theta vector (radians)
%       cspace.resolution   - same as env.resolution
%       cspace.robotMode    - which robot mode was used
%       cspace.freeSpace2D  - logical 2D free space (complement of grid2D)

    if nargin < 3
        nTheta = 36;  % 10 degree increments
    end
    
    fprintf('[generate_cspace] Building C-space for robot mode: %s ...\n', robot.mode);
    
    res = env.resolution;
    xVec = env.xRange(1):res:env.xRange(2);
    yVec = env.yRange(1):res:env.yRange(2);
    nx = length(xVec);
    ny = length(yVec);
    
    cspace.xVec = xVec;
    cspace.yVec = yVec;
    cspace.resolution = res;
    cspace.robotMode = robot.mode;
    
    %% --- Method 1: 2D C-space via Minkowski Sum (theta = 0) ---
    % Expand obstacles by robot bounding radius using dilation
    fprintf('  Computing 2D C-space (Minkowski expansion) ...\n');
    
    % Number of grid cells for robot radius
    r_cells = ceil(robot.radius / res);
    
    % Create circular structuring element
    [dx, dy] = meshgrid(-r_cells:r_cells, -r_cells:r_cells);
    se = (dx.^2 + dy.^2) <= r_cells^2;
    
    % Dilate obstacle grid
    grid2D = imdilate_manual(env.grid, se);
    
    % Also add boundary walls to C-space
    grid2D(1:r_cells, :) = true;
    grid2D(end-r_cells+1:end, :) = true;
    grid2D(:, 1:r_cells) = true;
    grid2D(:, end-r_cells+1:end) = true;
    
    cspace.grid2D = grid2D;
    cspace.freeSpace2D = ~grid2D;
    
    fprintf('  2D C-space: %d x %d | Free cells: %d / %d (%.1f%%)\n', ...
        ny, nx, sum(~grid2D(:)), numel(grid2D), 100*sum(~grid2D(:))/numel(grid2D));
    
    %% --- Method 2: 3D C-space (x, y, theta) - Sampled ---
    % This is more expensive, so we use a coarser resolution
    fprintf('  Computing 3D C-space (%d theta slices) ...\n', nTheta);
    
    thetaVec = linspace(0, 2*pi, nTheta+1);
    thetaVec(end) = [];  % remove duplicate 2*pi
    cspace.thetaVec = thetaVec;
    
    % Use coarser grid for 3D to save memory
    coarse_factor = 4;
    xVec_c = xVec(1:coarse_factor:end);
    yVec_c = yVec(1:coarse_factor:end);
    nx_c = length(xVec_c);
    ny_c = length(yVec_c);
    
    grid3D = false(ny_c, nx_c, nTheta);
    
    for kt = 1:nTheta
        theta = thetaVec(kt);
        % Get rotated footprint vertices
        verts = robot.getFootprint(0, 0, theta);
        
        % For each grid cell, check if robot placed there collides
        for iy = 1:ny_c
            for ix = 1:nx_c
                xc = xVec_c(ix);
                yc = yVec_c(iy);
                
                % Translate footprint
                fp = verts;
                fp(:,1) = fp(:,1) + xc;
                fp(:,2) = fp(:,2) + yc;
                
                % Check if any vertex is in obstacle
                if check_collision(fp, env)
                    grid3D(iy, ix, kt) = true;
                end
            end
        end
        
        if mod(kt, 6) == 0
            fprintf('    theta slice %d/%d done (%.0f deg)\n', kt, nTheta, rad2deg(theta));
        end
    end
    
    cspace.grid3D = grid3D;
    cspace.xVec_coarse = xVec_c;
    cspace.yVec_coarse = yVec_c;
    
    fprintf('[generate_cspace] Done. 3D C-space: %d x %d x %d\n', ny_c, nx_c, nTheta);
end


function dilated = imdilate_manual(grid, se)
% Manual dilation without Image Processing Toolbox
    [m, n] = size(grid);
    [sm, sn] = size(se);
    padm = floor(sm/2);
    padn = floor(sn/2);
    
    % Pad grid
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


function collides = check_collision(footprint_world, env)
% Check if any point of the robot footprint is inside an obstacle
    collides = false;
    
    for k = 1:size(footprint_world, 1)
        px = footprint_world(k, 1);
        py = footprint_world(k, 2);
        
        % Check bounds
        if px < env.xRange(1) || px > env.xRange(2) || ...
           py < env.yRange(1) || py > env.yRange(2)
            collides = true;
            return;
        end
        
        % Convert to grid index
        ci = round((px - env.xRange(1)) / env.resolution) + 1;
        ri = round((py - env.yRange(1)) / env.resolution) + 1;
        
        [nr, nc] = size(env.grid);
        ci = max(1, min(nc, ci));
        ri = max(1, min(nr, ri));
        
        if env.grid(ri, ci)
            collides = true;
            return;
        end
    end
    
    % Also check interior points along edges
    nv = size(footprint_world, 1);
    for k = 1:nv
        k2 = mod(k, nv) + 1;
        p1 = footprint_world(k, :);
        p2 = footprint_world(k2, :);
        
        nSamples = max(3, ceil(norm(p2-p1) / env.resolution));
        for s = 0:nSamples
            t = s / nSamples;
            px = p1(1) + t*(p2(1)-p1(1));
            py = p1(2) + t*(p2(2)-p1(2));
            
            ci = round((px - env.xRange(1)) / env.resolution) + 1;
            ri = round((py - env.yRange(1)) / env.resolution) + 1;
            
            [nr, nc] = size(env.grid);
            ci = max(1, min(nc, ci));
            ri = max(1, min(nr, ri));
            
            if env.grid(ri, ci)
                collides = true;
                return;
            end
        end
    end
end
