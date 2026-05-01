function poly_out = halton_robot_model(q)
% HALTON_ROBOT_MODEL Generates the 2D bounding footprint for the morphing robot
% Input:
%   q = [x, y, theta, d1, d2]
%   where d1 is actuator extension (0 to 1) 
%   and d2 is finger openness (0 to 1)
% Output:
%   poly_out : A single unioned polyshape representing the robot footprint

    if length(q) < 5
        % Defaults for d1 and d2 if only 3D global state is provided
        q = [q(1), q(2), q(3), 0, 0];
    end

    x = q(1); y = q(2); theta = q(3);
    d1 = q(4); % 0 to 1 (actuator extension)
    d2 = q(5); % 0 to 1 (finger openness: 1=wide open, 0=closed gap)
    
    % Robot dimensions matching a smaller morphing footprint (0.8 x 0.8)
    baseL = 0.8; baseW = 0.8;
    actLen = 0.6; fingL = 0.2;
    closedGap = 0.5; fingGap = 0.8;
    
    % --- Base vertices (local frame, center at 0,0) ---
    L2 = baseL / 2; W2 = baseW / 2;
    vBase = [-L2 -W2; L2 -W2; L2 W2; -L2 W2];
    
    % --- Actuator Extension (assuming front is +X) ---
    stroke = d1 * actLen;
    rodW = 0.15; % Actuator rod width
    vRod = [L2-0.5, -rodW/2; L2+stroke, -rodW/2; L2+stroke, rodW/2; L2-0.5, rodW/2];
    
    % --- Fingers ---
    gap = closedGap + d2 * (fingGap - closedGap);
    fW = 0.2; % width of finger prong
    % Left finger (positive Y)
    vFingL = [L2+stroke, gap/2-fW/2; L2+stroke+fingL, gap/2-fW/2; L2+stroke+fingL, gap/2+fW/2; L2+stroke, gap/2+fW/2];
    % Right finger (negative Y)
    vFingR = [L2+stroke, -gap/2-fW/2; L2+stroke+fingL, -gap/2-fW/2; L2+stroke+fingL, -gap/2+fW/2; L2+stroke, -gap/2+fW/2];
    
    % Rotate and Translate all parts
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    
    vBase_W =  (R * vBase')'  + [x, y];
    vRod_W =   (R * vRod')'   + [x, y];
    vFingL_W = (R * vFingL')' + [x, y];
    vFingR_W = (R * vFingR')' + [x, y];
    
    % Create polyshapes
    % In MATLAB 2017b+, polyshape ensures proper orientation and cleanup
    pBase = polyshape(vBase_W(:,1), vBase_W(:,2));
    pRod = polyshape(vRod_W(:,1), vRod_W(:,2));
    pFingL = polyshape(vFingL_W(:,1), vFingL_W(:,2));
    pFingR = polyshape(vFingR_W(:,1), vFingR_W(:,2));
    
    % Combine into single polyshape
    poly_out = union([pBase, pRod, pFingL, pFingR]);
end
