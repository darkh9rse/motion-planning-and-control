function [path, tree] = build_halton_rrt_star(start_q, goal_q, env, max_nodes)
% BUILD_HALTON_RRT_STAR Constructs an RRT* tree using Halton samples
% Input:
%   start_q: [x, y, theta]
%   goal_q:  [x, y, theta]
%   env: Environment struct (needs xRange, yRange, obstacles)
%   max_nodes: Maximum points to sample from Halton sequence
% Output:
%   path: Nx3 matrix of the path
%   tree: Struct containing nodes and parents for visualization

    % Convert env obstacles to polyshapes
    obs_poly = [];
    for i = 1:size(env.obstacles, 1)
        ox = env.obstacles(i,1); oy = env.obstacles(i,2);
        ow = env.obstacles(i,3); oh = env.obstacles(i,4);
        obs_poly = [obs_poly; polyshape([ox ox+ow ox+ow ox], [oy oy oy+oh oy+oh])];
    end
    
    % Outer walls as obstacles to keep robot in bounds
    xmin = env.xRange(1); xmax = env.xRange(2);
    ymin = env.yRange(1); ymax = env.yRange(2);
    w1 = polyshape([xmin-5 xmin-5 xmax+5 xmax+5], [ymin-5 ymin ymin ymin-5]); % bottom
    w2 = polyshape([xmin-5 xmin-5 xmax+5 xmax+5], [ymax ymax+5 ymax+5 ymax]); % top
    w3 = polyshape([xmin-5 xmin-5 xmin xmin], [ymin-5 ymax+5 ymax+5 ymin-5]); % left
    w4 = polyshape([xmax xmax xmax+5 xmax+5], [ymin-5 ymax+5 ymax+5 ymin-5]); % right
    obs_poly = [obs_poly; w1; w2; w3; w4];
    
    % Combine all obstacles into ONE polyshape to accelerate `overlaps` checks
    obs_poly = union(obs_poly);

    % Verify start and goal
    if check_collision_polyhedron(start_q, obs_poly)
        error('Start state is in collision.');
    end
    if check_collision_polyhedron(goal_q, obs_poly)
        error('Goal state is in collision.');
    end

    % Pre-allocate generated Halton points
    samples = generate_halton_samples_3d(max_nodes, env.xRange, env.yRange);
    
    nodes = zeros(max_nodes+1, 3);
    nodes(1,:) = start_q;
    
    parents = zeros(max_nodes+1, 1);
    costs = zeros(max_nodes+1, 1);
    
    node_count = 1;
    
    step_size = 0.5;      % Max extension step
    search_radius = 2.5;  % RRT* rewiring radius
    
    goal_reached = false;
    goal_idx = -1;
    
    disp('Building RRT* tree using Halton samples...');
    for i = 1:max_nodes
        q_rand = samples(i, :);
        
        % Bias to goal occasionally
        if mod(i, 20) == 0
            q_rand = goal_q;
        end
        
        % Find nearest node (using un-normalized angles roughly)
        dist_sq = (nodes(1:node_count, 1) - q_rand(1)).^2 + ...
                  (nodes(1:node_count, 2) - q_rand(2)).^2 + ...
                  1.0 * (angdiff_local(nodes(1:node_count, 3), q_rand(3))).^2;
        [~, min_idx] = min(dist_sq);
        q_near = nodes(min_idx, :);
        
        % Steer towards q_rand
        dist_line = norm([q_rand(1)-q_near(1), q_rand(2)-q_near(2)]);
        d_th = angdiff_local(q_near(3), q_rand(3));
        
        if dist_line > step_size
            t = step_size / dist_line;
            q_new = q_near;
            q_new(1) = q_near(1) + t * (q_rand(1)-q_near(1));
            q_new(2) = q_near(2) + t * (q_rand(2)-q_near(2));
            q_new(3) = q_near(3) + t * d_th;
        else
            q_new = q_rand;
        end
        
        % Check collision
        if ~check_collision_polyhedron(q_near, obs_poly, q_new)
            node_count = node_count + 1;
            nodes(node_count, :) = q_new;
            
            % RRT* Rewiring
            % Find neighbors in radius
            neighbor_dist_sq = (nodes(1:node_count-1, 1) - q_new(1)).^2 + ...
                               (nodes(1:node_count-1, 2) - q_new(2)).^2 + ...
                               1.0*(angdiff_local(nodes(1:node_count-1, 3), q_new(3))).^2;
            neighbors = find(neighbor_dist_sq < search_radius^2);
            
            % Choose best parent
            best_parent = min_idx;
            best_cost = costs(min_idx) + sqrt((q_new(1)-q_near(1))^2 + (q_new(2)-q_near(2))^2 + (angdiff_local(q_new(3), q_near(3)))^2);
            
            for j = 1:length(neighbors)
                n_idx = neighbors(j);
                q_n = nodes(n_idx, :);
                c_test = costs(n_idx) + sqrt((q_new(1)-q_n(1))^2 + (q_new(2)-q_n(2))^2 + (angdiff_local(q_new(3), q_n(3)))^2);
                if c_test < best_cost
                    if ~check_collision_polyhedron(q_n, obs_poly, q_new)
                        best_parent = n_idx;
                        best_cost = c_test;
                    end
                end
            end
            
            parents(node_count) = best_parent;
            costs(node_count) = best_cost;
            
            % Rewire neighbors
            for j = 1:length(neighbors)
                n_idx = neighbors(j);
                q_n = nodes(n_idx, :);
                c_test = costs(node_count) + sqrt((q_new(1)-q_n(1))^2 + (q_new(2)-q_n(2))^2 + (angdiff_local(q_new(3), q_n(3)))^2);
                if c_test < costs(n_idx)
                    if ~check_collision_polyhedron(q_new, obs_poly, q_n)
                        parents(n_idx) = node_count;
                        costs(n_idx) = c_test;
                    end
                end
            end
            
            % Check if close to goal
            dist_to_goal = sqrt((q_new(1)-goal_q(1))^2 + (q_new(2)-goal_q(2))^2 + (angdiff_local(q_new(3), goal_q(3)))^2);
            if dist_to_goal < 0.5 && ~check_collision_polyhedron(q_new, obs_poly, goal_q)
                if ~goal_reached || costs(node_count) < costs(goal_idx)
                    goal_idx = node_count;
                    goal_reached = true; 
                end
            end
        end
        
        if mod(i, 1000) == 0
            fprintf('RRT* iteration %d / %d\n', i, max_nodes);
        end
    end
    
    if ~goal_reached
        warning('RRT* did not reach the goal within maximum nodes.');
        path = [];
    else
        % Backtrack to get path
        path = [goal_q];
        curr = goal_idx;
        while curr > 1
            path = [nodes(curr, :); path];
            curr = parents(curr);
        end
        path = [start_q; path];
        fprintf('Optimal path found with optimal cost metric.\n');
    end
    
    tree.nodes = nodes(1:node_count, :);
    tree.parents = parents(1:node_count);
end

function d = angdiff_local(theta1, theta2)
    d = mod((theta2 - theta1) + pi, 2*pi) - pi;
end
