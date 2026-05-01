function skel = generate_skeleton(cspace)
% GENERATE_SKELETON  Compute the skeleton (medial axis / Voronoi diagram)
%                    of the free C-space for path planning.
%
%   skel = generate_skeleton()
%   skel = generate_skeleton(cspace)
%
%   Uses the distance transform of the free space and then extracts the
%   topological skeleton (ridges of the distance field). This skeleton
%   serves as a roadmap: the robot prefers to travel along paths that
%   maximize clearance from obstacles.
%
%   Returns:
%       skel.distMap        - Distance transform of free space
%       skel.skelMap        - Binary skeleton image
%       skel.skelPoints     - [x y] world coordinates of skeleton pixels
%       skel.adjacency      - Sparse adjacency matrix of skeleton graph
%       skel.nodes          - Graph node positions [x y]
%       skel.clearance      - Clearance value at each skeleton point

    if nargin < 1 || isempty(cspace)
        fprintf('[generate_skeleton] No C-space input. Building default C-space...\n');
        env = warehouse_environment();
        robot = morphing_robot('default');
        cspace = generate_cspace(env, robot, 12);
    end

    requiredFields = {'freeSpace2D', 'resolution', 'xVec', 'yVec'};
    for i = 1:numel(requiredFields)
        if ~isfield(cspace, requiredFields{i})
            error('generate_skeleton:InvalidInput', ...
                'Input cspace is missing required field: %s', requiredFields{i});
        end
    end

    fprintf('[generate_skeleton] Computing skeleton of free C-space ...\n');
    
    freeSpace = cspace.freeSpace2D;
    res = cspace.resolution;
    xVec = cspace.xVec;
    yVec = cspace.yVec;
    
    %% --- Distance Transform ---
    % Compute distance of each free cell to nearest obstacle
    distMap = bwdist_manual(~freeSpace);
    distMap = distMap * res;  % convert to meters
    
    skel.distMap = distMap;
    
    fprintf('  Max clearance in free space: %.2f m\n', max(distMap(:)));
    
    %% --- Skeleton Extraction via Ridge Detection ---
    % The skeleton consists of points that are local maxima of the distance
    % transform along at least one direction (ridges).
    % We use a morphological thinning approach.
    
    skelMap = thinning(freeSpace);
    
    % Remove very short spurs
    skelMap = remove_spurs(skelMap, 5);
    
    skel.skelMap = skelMap;
    
    %% --- Extract Skeleton Point Coordinates ---
    [rows, cols] = find(skelMap);
    skelPoints = zeros(length(rows), 2);
    skelPoints(:,1) = xVec(min(cols, length(xVec)));  % x
    skelPoints(:,2) = yVec(min(rows, length(yVec)));  % y
    
    skel.skelPoints = skelPoints;
    
    % Clearance at each skeleton point
    clearance = zeros(length(rows), 1);
    for k = 1:length(rows)
        r = rows(k); c = cols(k);
        if r <= size(distMap,1) && c <= size(distMap,2)
            clearance(k) = distMap(r, c);
        end
    end
    skel.clearance = clearance;
    
    %% --- Build Graph from Skeleton ---
    % Each skeleton pixel is a node; 8-connected neighbors on skeleton are edges
    fprintf('  Building skeleton graph ...\n');
    
    nPts = length(rows);
    
    % Create a lookup map: pixel (r,c) -> index
    [ny, nx] = size(skelMap);
    idxMap = zeros(ny, nx);
    for k = 1:nPts
        idxMap(rows(k), cols(k)) = k;
    end
    
    % 8-connectivity offsets
    offsets = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];
    
    I = []; J = []; W = [];
    for k = 1:nPts
        r = rows(k);
        c = cols(k);
        for d = 1:8
            nr = r + offsets(d,1);
            nc = c + offsets(d,2);
            if nr >= 1 && nr <= ny && nc >= 1 && nc <= nx
                if idxMap(nr, nc) > 0
                    neighbor = idxMap(nr, nc);
                    dist = norm([offsets(d,1), offsets(d,2)]) * res;
                    I = [I; k]; %#ok<AGROW>
                    J = [J; neighbor]; %#ok<AGROW>
                    W = [W; dist]; %#ok<AGROW>
                end
            end
        end
    end
    
    skel.adjacency = sparse(I, J, W, nPts, nPts);
    skel.nodes = skelPoints;
    
    fprintf('[generate_skeleton] Skeleton: %d points, %d edges\n', nPts, length(I)/2);
end


%% ===================== Helper Functions =====================

function dist = bwdist_manual(binaryMask)
% Compute Euclidean distance transform without Image Processing Toolbox
% Uses a two-pass approximation (Rosenfeld & Pfaltz style with Euclidean metric)
    [m, n] = size(binaryMask);
    dist = inf(m, n);
    dist(binaryMask) = 0;
    
    % Forward pass
    for i = 1:m
        for j = 1:n
            if dist(i,j) > 0
                val = dist(i,j);
                if i > 1
                    val = min(val, dist(i-1,j) + 1);
                end
                if j > 1
                    val = min(val, dist(i,j-1) + 1);
                end
                if i > 1 && j > 1
                    val = min(val, dist(i-1,j-1) + 1.414);
                end
                if i > 1 && j < n
                    val = min(val, dist(i-1,j+1) + 1.414);
                end
                dist(i,j) = val;
            end
        end
    end
    
    % Backward pass
    for i = m:-1:1
        for j = n:-1:1
            if dist(i,j) > 0
                val = dist(i,j);
                if i < m
                    val = min(val, dist(i+1,j) + 1);
                end
                if j < n
                    val = min(val, dist(i,j+1) + 1);
                end
                if i < m && j < n
                    val = min(val, dist(i+1,j+1) + 1.414);
                end
                if i < m && j > 1
                    val = min(val, dist(i+1,j-1) + 1.414);
                end
                dist(i,j) = val;
            end
        end
    end
end


function thin = thinning(bw)
% Morphological thinning - iteratively remove border pixels
% Produces a skeleton (medial axis) of the binary shape
    thin = bw;
    changed = true;
    
    while changed
        changed = false;
        markers = false(size(thin));
        
        % Sub-iteration 1
        for i = 2:size(thin,1)-1
            for j = 2:size(thin,2)-1
                if thin(i,j)
                    % Get 3x3 neighborhood
                    P = [thin(i-1,j) thin(i-1,j+1) thin(i,j+1) thin(i+1,j+1) ...
                         thin(i+1,j) thin(i+1,j-1) thin(i,j-1) thin(i-1,j-1)];
                    
                    B = sum(P);  % number of nonzero neighbors
                    
                    % Count 0->1 transitions in ordered sequence
                    A = 0;
                    for k = 1:7
                        if P(k) == 0 && P(k+1) == 1
                            A = A + 1;
                        end
                    end
                    if P(8) == 0 && P(1) == 1
                        A = A + 1;
                    end
                    
                    if A == 1 && B >= 2 && B <= 6 && ...
                       P(1)*P(3)*P(5) == 0 && P(3)*P(5)*P(7) == 0
                        markers(i,j) = true;
                        changed = true;
                    end
                end
            end
        end
        thin(markers) = false;
        
        markers = false(size(thin));
        
        % Sub-iteration 2
        for i = 2:size(thin,1)-1
            for j = 2:size(thin,2)-1
                if thin(i,j)
                    P = [thin(i-1,j) thin(i-1,j+1) thin(i,j+1) thin(i+1,j+1) ...
                         thin(i+1,j) thin(i+1,j-1) thin(i,j-1) thin(i-1,j-1)];
                    
                    B = sum(P);
                    A = 0;
                    for k = 1:7
                        if P(k) == 0 && P(k+1) == 1
                            A = A + 1;
                        end
                    end
                    if P(8) == 0 && P(1) == 1
                        A = A + 1;
                    end
                    
                    if A == 1 && B >= 2 && B <= 6 && ...
                       P(1)*P(3)*P(7) == 0 && P(1)*P(5)*P(7) == 0
                        markers(i,j) = true;
                        changed = true;
                    end
                end
            end
        end
        thin(markers) = false;
    end
end


function skelClean = remove_spurs(skelMap, spurLength)
% Remove short branches (spurs) from skeleton
    skelClean = skelMap;
    
    for iter = 1:spurLength
        endpoints = false(size(skelClean));
        for i = 2:size(skelClean,1)-1
            for j = 2:size(skelClean,2)-1
                if skelClean(i,j)
                    neighbors = sum(sum(skelClean(i-1:i+1, j-1:j+1))) - 1;
                    if neighbors == 1
                        endpoints(i,j) = true;
                    end
                end
            end
        end
        skelClean(endpoints) = false;
    end
end
