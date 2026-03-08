function robot = morphing_robot(mode)
% MORPHING_ROBOT  Define the morphing robot with multiple configurations.
%
%   robot = morphing_robot(mode)
%
%   The robot can morph between different shapes to navigate tight spaces.
%
%   Modes:
%       'default'   - Standard square shape (good for open areas)
%       'narrow'    - Tall & narrow shape  (good for narrow aisles)
%       'flat'      - Wide & flat shape    (good for low clearance)
%       'compact'   - Smallest circular    (good for tight turns)
%
%   Returns a struct with:
%       robot.mode          - Current mode string
%       robot.width         - Width in meters
%       robot.height        - Height in meters
%       robot.radius        - Equivalent bounding circle radius
%       robot.vertices      - Nx2 polygon vertices (local frame)
%       robot.color         - Display color
%       robot.maxSpeed      - Max speed in that mode (m/s)
%       robot.morphTime     - Time to morph to this config (seconds)
%       robot.allModes      - Cell array of all available modes
%       robot.getFootprint  - Function handle: getFootprint(x,y,theta)

    if nargin < 1
        mode = 'default';
    end
    
    robot.allModes = {'default', 'narrow', 'flat', 'compact'};
    robot.mode = mode;
    
    switch lower(mode)
        case 'default'
            robot.width  = 0.8;
            robot.height = 0.8;
            robot.color  = [0.2 0.6 1.0];
            robot.maxSpeed = 1.5;
            robot.morphTime = 0;
            % Square footprint
            w2 = robot.width/2; h2 = robot.height/2;
            robot.vertices = [-w2 -h2; w2 -h2; w2 h2; -w2 h2];
            
        case 'narrow'
            robot.width  = 0.4;
            robot.height = 1.2;
            robot.color  = [1.0 0.5 0.2];
            robot.maxSpeed = 1.0;
            robot.morphTime = 1.5;
            % Tall narrow rectangle
            w2 = robot.width/2; h2 = robot.height/2;
            robot.vertices = [-w2 -h2; w2 -h2; w2 h2; -w2 h2];
            
        case 'flat'
            robot.width  = 1.2;
            robot.height = 0.4;
            robot.color  = [0.2 0.9 0.3];
            robot.maxSpeed = 1.0;
            robot.morphTime = 1.5;
            % Wide flat rectangle
            w2 = robot.width/2; h2 = robot.height/2;
            robot.vertices = [-w2 -h2; w2 -h2; w2 h2; -w2 h2];
            
        case 'compact'
            robot.width  = 0.5;
            robot.height = 0.5;
            robot.color  = [0.9 0.2 0.8];
            robot.maxSpeed = 0.8;
            robot.morphTime = 2.0;
            % Approximate circle with polygon (16-gon)
            npts = 16;
            theta_pts = linspace(0, 2*pi, npts+1); theta_pts(end) = [];
            robot.vertices = 0.25 * [cos(theta_pts)', sin(theta_pts)'];
            
        otherwise
            error('Unknown robot mode: %s. Use: default, narrow, flat, compact.', mode);
    end
    
    % Bounding radius (for C-space expansion)
    robot.radius = sqrt((robot.width/2)^2 + (robot.height/2)^2);
    
    % Function to compute world-frame footprint at pose (x, y, theta)
    robot.getFootprint = @(x, y, theta) transform_footprint(robot.vertices, x, y, theta);
    
    fprintf('[morphing_robot] Mode: %-8s | Size: %.1f x %.1f m | Radius: %.2f m\n', ...
        robot.mode, robot.width, robot.height, robot.radius);
end


function fp = transform_footprint(vertices, x, y, theta)
% Rotate and translate the local footprint to world coordinates
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    fp = (R * vertices')';
    fp(:,1) = fp(:,1) + x;
    fp(:,2) = fp(:,2) + y;
end
