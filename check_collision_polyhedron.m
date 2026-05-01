function inCollision = check_collision_polyhedron(q1, obstacles_poly, q2)
% CHECK_COLLISION_POLYHEDRON Tests robot footprint for intersection with obstacles.
%
%   col = check_collision_polyhedron(q1, env_polygons)
%       Checks the state q1 for collision.
%   col = check_collision_polyhedron(q1, env_polygons, q2)
%       Checks the edge between q1 and q2 for collisions by linearly interpolating.

    if nargin < 3
        q2 = [];
    end

    inCollision = false;

    if ~isempty(q2)
        % Edge collision check: interpolate linearly between q1 and q2
        dist_x = q2(1) - q1(1);
        dist_y = q2(2) - q1(2);
        d_theta = wrapToPi_local(q2(3) - q1(3));
        
        linear_dist = norm([dist_x, dist_y]);

        % Step sizes: Every 0.1m or 0.1 rad
        n_steps = max(ceil(linear_dist / 0.1), ceil(abs(d_theta) / 0.1));
        n_steps = max(2, n_steps);

        for i = 1:n_steps
            t = (i-1)/(n_steps-1);
            qi = q1;
            qi(1) = q1(1) + t * dist_x;
            qi(2) = q1(2) + t * dist_y;
            qi(3) = q1(3) + t * d_theta;

            if check_state_collision(qi, obstacles_poly)
                inCollision = true;
                return;
            end
        end
    else
        % State collision check
        inCollision = check_state_collision(q1, obstacles_poly);
    end
end

function col = check_state_collision(q, obstacles_poly)
    col = false;
    % Uses 2D projection (x, y, theta) defaulting d1, d2 to 0
    poly_robot = halton_robot_model(q);
    
    for i = 1:numel(obstacles_poly)
        if overlaps(poly_robot, obstacles_poly(i))
            col = true;
            return;
        end
    end
end

function d = wrapToPi_local(a)
    d = mod(a + pi, 2*pi) - pi;
end
