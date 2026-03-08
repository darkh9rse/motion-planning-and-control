function path = plan_path(skel, startPos, goalPos, robot)
% PLAN_PATH  Plan a path on the skeleton graph using Dijkstra's algorithm.
%
%   path = plan_path(skel, startPos, goalPos, robot)
%
%   Finds the shortest path on the skeleton graph from startPos to goalPos.
%   The path is weighted by distance, with a bonus for higher clearance
%   (the robot prefers paths where it has more room to maneuver).
%
%   Inputs:
%       skel      - Skeleton struct from generate_skeleton()
%       startPos  - [x y] start position
%       goalPos   - [x y] goal position
%       robot     - Robot struct from morphing_robot()
%
%   Returns:
%       path.waypoints    - Nx2 path waypoints [x y]
%       path.totalDist    - Total path length (meters)
%       path.clearances   - Clearance at each waypoint
%       path.morphPlan    - Suggested morphing plan along the path

    fprintf('[plan_path] Planning from (%.1f, %.1f) to (%.1f, %.1f) ...\n', ...
        startPos(1), startPos(2), goalPos(1), goalPos(2));
    
    nodes = skel.nodes;
    nNodes = size(nodes, 1);
    
    if nNodes == 0
        warning('Skeleton has no nodes. Returning direct path.');
        path.waypoints = [startPos; goalPos];
        path.totalDist = norm(goalPos - startPos);
        path.clearances = [0; 0];
        path.morphPlan = {'default'; 'default'};
        return;
    end
    
    %% --- Find nearest skeleton nodes to start and goal ---
    dStart = vecnorm(nodes - startPos, 2, 2);
    dGoal  = vecnorm(nodes - goalPos, 2, 2);
    
    [~, startNode] = min(dStart);
    [~, goalNode]  = min(dGoal);
    
    fprintf('  Start node: %d (%.2f m away) | Goal node: %d (%.2f m away)\n', ...
        startNode, dStart(startNode), goalNode, dGoal(goalNode));
    
    %% --- Dijkstra's Algorithm ---
    adj = skel.adjacency;
    
    dist = inf(nNodes, 1);
    prev = zeros(nNodes, 1);
    visited = false(nNodes, 1);
    
    dist(startNode) = 0;
    
    for iter = 1:nNodes
        % Find unvisited node with minimum distance
        tempDist = dist;
        tempDist(visited) = inf;
        [minDist, u] = min(tempDist);
        
        if isinf(minDist) || u == goalNode
            break;
        end
        
        visited(u) = true;
        
        % Relax neighbors
        neighbors = find(adj(u, :));
        for k = 1:length(neighbors)
            v = neighbors(k);
            if ~visited(v)
                edgeWeight = adj(u, v);
                
                % Penalize low-clearance edges (clearance < robot radius)
                avgClearance = (skel.clearance(u) + skel.clearance(v)) / 2;
                if avgClearance < robot.radius
                    clearancePenalty = 5.0 * (robot.radius - avgClearance);
                else
                    clearancePenalty = 0;
                end
                
                altDist = dist(u) + edgeWeight + clearancePenalty;
                if altDist < dist(v)
                    dist(v) = altDist;
                    prev(v) = u;
                end
            end
        end
    end
    
    %% --- Reconstruct Path ---
    if dist(goalNode) == inf
        warning('No path found on skeleton graph!');
        path.waypoints = [startPos; goalPos];
        path.totalDist = inf;
        path.clearances = [0; 0];
        path.morphPlan = {'default'; 'default'};
        return;
    end
    
    % Backtrace
    nodeSeq = goalNode;
    current = goalNode;
    while current ~= startNode && prev(current) ~= 0
        current = prev(current);
        nodeSeq = [current; nodeSeq]; %#ok<AGROW>
    end
    
    % Build waypoint list: start -> skeleton nodes -> goal
    waypoints = [startPos; nodes(nodeSeq, :); goalPos];
    
    % Clearances
    clearances = zeros(size(waypoints, 1), 1);
    clearances(1) = 0;
    clearances(end) = 0;
    for k = 1:length(nodeSeq)
        clearances(k+1) = skel.clearance(nodeSeq(k));
    end
    
    % Total distance
    totalDist = 0;
    for k = 1:size(waypoints,1)-1
        totalDist = totalDist + norm(waypoints(k+1,:) - waypoints(k,:));
    end
    
    path.waypoints  = waypoints;
    path.totalDist  = totalDist;
    path.clearances = clearances;
    
    %% --- Generate Morphing Plan ---
    % Decide robot mode based on local clearance
    morphPlan = cell(size(waypoints, 1), 1);
    for k = 1:size(waypoints, 1)
        c = clearances(k);
        if c > 1.0
            morphPlan{k} = 'default';    % plenty of room
        elseif c > 0.6
            morphPlan{k} = 'narrow';     % tight aisle, go narrow
        elseif c > 0.4
            morphPlan{k} = 'compact';    % very tight, go compact
        else
            morphPlan{k} = 'compact';    % minimal clearance
        end
    end
    path.morphPlan = morphPlan;
    
    fprintf('[plan_path] Path found: %d waypoints, %.2f m total distance\n', ...
        size(waypoints,1), totalDist);
end
