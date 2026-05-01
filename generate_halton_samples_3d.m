function pts = generate_halton_samples_3d(num_samples, xBounds, yBounds)
% GENERATE_HALTON_SAMPLES_3D Generates 3D sample points (x, y, theta) 
% using the Halton sequence generator.
%
% Inputs:
%   num_samples: Total number of (x,y,theta) tuples to generate
%   xBounds    : [xmin, xmax] allowed range of x
%   yBounds    : [ymin, ymax] allowed range of y
% 
% Outputs:
%   pts        : Nx3 array of [x, y, theta]

    % Create Halton sequence set in 3 Dimensions
    hset = haltonset(3, 'Skip', 100); 
    
    % Scramble for more randomized uniformity if necessary
    hset = scramble(hset, 'RR2');
    
    % Extract the net matrix of samples
    val = net(hset, num_samples); 
    
    % Map to configuration space bounds
    pts = zeros(num_samples, 3);
    pts(:,1) = xBounds(1) + val(:,1) * (xBounds(2) - xBounds(1));
    pts(:,2) = yBounds(1) + val(:,2) * (yBounds(2) - yBounds(1));
    
    % Theta mapped from -pi to pi
    pts(:,3) = -pi + val(:,3) * (2*pi);
end
