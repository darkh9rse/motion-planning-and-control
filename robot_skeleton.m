function robot_skeleton()
% ROBOT_SKELETON  3-D animated skeleton of the morphing warehouse AMR.
%   Matches the SolidWorks CAD reference:
%     - Compact / short body (thin slab chassis)
%     - Big spoked wheels with axle along Y (visible from side)
%     - Bottom-hinged inclined ramp (tip reaches ground)
%     - Linear actuator mounted HORIZONTALLY through body walls
%     - Fingers attached to actuator rod ends (L-shaped, hang down to ramp level)
%
%   Run:  robot_skeleton

    clc; close all;

    %% ---------- dimensions ----------
    R = robot_dims();

    %% ---------- scene ----------
    fig = figure('Name','AMR 3-D Skeleton','Color','w',...
                 'Position',[100 50 1200 700]);
    ax  = axes('Parent',fig); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    axis(ax,'equal');
    xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
    view(ax, 150, 20);
    light(ax,'Position',[15 -12 18]);
    light(ax,'Position',[-8  10  8],'Style','local');
    lighting(ax,'gouraud');

    %% ---------- ground plane ----------
    fill3(ax,[-5 35 35 -5],[-8 -8 8 8],[0 0 0 0],...
          [0.88 0.88 0.88],'FaceAlpha',0.3,'EdgeColor','none');

    %% ---------- box (cargo) ----------
    boxW = 1.6; boxH = 1.6; boxD = 1.6;
    boxX = 22;  boxY = 0;   boxZ = 0;
    draw_cargo(ax, boxX, boxY, boxZ, boxD, boxW, boxH, [0.85 0.55 0.25]);
    text(ax, boxX, boxY, boxZ+boxH+0.3, 'CARGO',...
         'HorizontalAlignment','center','FontWeight','bold','FontSize',9);

    %% ---------- title handle ----------
    ht = title(ax,'Phase : Idle','FontSize',13);

    %% ========== ANIMATION ==========
    fps = 15;  dt = 1/fps;

    % ----- Phase 0 : draw robot at rest -----
    startX = 4;
    draw_robot(ax, R, startX, 0, 0, 0);
    pause(0.4);

    % ----- Phase 1 : approach box -----
    % When ramp fully open, tip touches ground at hingeX + rampLen*sin(a_max)
    a_max  = acos(-R.baseZ / R.rampLen);
    tipReach = R.rampLen * sin(a_max);
    gap    = 0.25;
    stopX  = boxX - gap - tipReach - R.baseL/2;
    nF     = round(1.2*fps);
    xs     = linspace(startX, stopX, nF);
    set(ht,'String','Phase 1 : Approach');
    for i = 1:nF
        delete(findobj(ax,'Tag','robot'));
        draw_robot(ax, R, xs(i), 0, 0, 0);
        drawnow; pause(dt);
    end

    % ----- Phase 2 : open gate → inclined ramp -----
    nF   = round(0.8*fps);
    angs = linspace(0, 90, nF);
    set(ht,'String','Phase 2 : Lower Gate / Ramp');
    for i = 1:nF
        delete(findobj(ax,'Tag','robot'));
        draw_robot(ax, R, stopX, 0, angs(i), 0);
        drawnow; pause(dt);
    end

    % ----- Phase 3 : extend actuator (fingers open, reaching out) -----
    nF   = round(0.7*fps);
    exts = linspace(0, 1, nF);
    set(ht,'String','Phase 3 : Extend Actuator');
    for i = 1:nF
        delete(findobj(ax,'Tag','robot'));
        draw_robot(ax, R, stopX, 0, 90, exts(i), 1);   % fingOpen=1
        drawnow; pause(dt);
    end

    % ----- Phase 3b : close fingers around box -----
    nF   = round(0.5*fps);
    fCloses = linspace(1, 0, nF);
    set(ht,'String','Phase 3b : Close Fingers');
    for i = 1:nF
        delete(findobj(ax,'Tag','robot'));
        draw_robot(ax, R, stopX, 0, 90, 1, fCloses(i));  % actExt=1, fingOpen 1→0
        drawnow; pause(dt);
    end

    % ----- Phase 4 : grip box (flash) -----
    set(ht,'String','Phase 4 : Grip Box');
    for k = 1:4
        delete(findobj(ax,'Tag','cargo'));
        if mod(k,2)==1
            draw_cargo(ax,boxX,boxY,boxZ,boxD,boxW,boxH,[1 0.3 0.3]);
        else
            draw_cargo(ax,boxX,boxY,boxZ,boxD,boxW,boxH,[0.85 0.55 0.25]);
        end
        drawnow; pause(0.15);
    end

    % ----- Phase 5 : retract actuator → pull box up ramp (fingers closed) -----
    nF   = round(1.0*fps);
    exts = linspace(1, 0, nF);
    hingeX = stopX + R.baseL/2;
    hingeZ = R.baseZ;
    tipX0  = hingeX + R.rampLen * sin(a_max);
    tipZ0  = 0;
    set(ht,'String','Phase 5 : Retract Actuator (pull box)');
    for i = 1:nF
        frac = 1 - exts(i);
        bx   = tipX0 - frac*(tipX0 - hingeX);
        bz   = tipZ0 + frac*(hingeZ - tipZ0);
        delete(findobj(ax,'Tag','robot'));
        delete(findobj(ax,'Tag','cargo'));
        draw_robot(ax, R, stopX, 0, 90, exts(i), 0);    % fingOpen=0 (closed)
        draw_cargo(ax, bx, boxY, bz, boxD, boxW, boxH, [0.85 0.55 0.25]);
        drawnow; pause(dt);
    end

    % ----- Phase 6 : close gate -----
    nF   = round(0.7*fps);
    angs = linspace(90, 0, nF);
    insideX = hingeX - boxD - 0.15;
    insideZ = R.baseZ + 0.03;
    delete(findobj(ax,'Tag','cargo'));
    draw_cargo(ax, insideX, boxY, insideZ, boxD, boxW, boxH, [0.85 0.55 0.25]);
    set(ht,'String','Phase 6 : Close Gate');
    for i = 1:nF
        delete(findobj(ax,'Tag','robot'));
        draw_robot(ax, R, stopX, 0, angs(i), 0);
        drawnow; pause(dt);
    end

    % ----- Phase 7 : conveyor moves box deeper -----
    nF   = round(0.7*fps);
    dest = stopX - R.baseL/2 + 0.3;
    bxs  = linspace(insideX, dest, nF);
    set(ht,'String','Phase 7 : Conveyor Belt Active');
    for i = 1:nF
        delete(findobj(ax,'Tag','cargo'));
        draw_cargo(ax, bxs(i), boxY, insideZ, boxD, boxW, boxH, [0.85 0.55 0.25]);
        drawnow; pause(dt);
    end

    % ----- Phase 8 : drive to drop-off location -----
    dropX = stopX - 12;
    nF = round(1.0*fps);
    xs = linspace(stopX, dropX, nF);
    set(ht,'String','Phase 8 : Drive to Drop-off');
    for i = 1:nF
        delete(findobj(ax,'Tag','robot'));
        delete(findobj(ax,'Tag','cargo'));
        draw_robot(ax, R, xs(i), 0, 0, 0);
        draw_cargo(ax, bxs(end)-(stopX-xs(i)), boxY, insideZ,...
                   boxD, boxW, boxH, [0.85 0.55 0.25]);
        drawnow; pause(dt);
    end
    cargoInsideX = bxs(end) - (stopX - dropX);

    % ----- Phase 9 : open ramp at drop-off -----
    nF   = round(0.8*fps);
    angs = linspace(0, 90, nF);
    set(ht,'String','Phase 9 : Open Ramp (drop-off)');
    for i = 1:nF
        delete(findobj(ax,'Tag','robot'));
        draw_robot(ax, R, dropX, 0, angs(i), 0, 0);    % fingers still closed
        drawnow; pause(dt);
    end

    % ----- Phase 9b : open fingers to release cargo -----
    nF   = round(0.5*fps);
    fOpens = linspace(0, 1, nF);
    set(ht,'String','Phase 9b : Open Fingers (release)');
    for i = 1:nF
        delete(findobj(ax,'Tag','robot'));
        draw_robot(ax, R, dropX, 0, 90, 0, fOpens(i));  % fingOpen 0→1
        drawnow; pause(dt);
    end

    % ----- Phase 10 : cargo slides out down the ramp -----
    nF = round(0.8*fps);
    dropHingeX = dropX + R.baseL/2;
    dropHingeZ = R.baseZ;
    a_max_drop = acos(-R.baseZ / R.rampLen);
    dropTipX   = dropHingeX + R.rampLen * sin(a_max_drop);
    dropTipZ   = 0;
    set(ht,'String','Phase 10 : Cargo Slides Out');
    for i = 1:nF
        frac = i / nF;   % 0→1 cargo moves from inside to ramp tip
        bx = cargoInsideX + frac * (dropTipX + 0.3 - cargoInsideX);
        bz = insideZ      + frac * (dropTipZ - insideZ);
        delete(findobj(ax,'Tag','cargo'));
        draw_cargo(ax, bx, boxY, bz, boxD, boxW, boxH, [0.85 0.55 0.25]);
        drawnow; pause(dt);
    end

    % ----- Phase 11 : cargo lands on ground -----
    delete(findobj(ax,'Tag','cargo'));
    landX = dropTipX + 0.5;
    draw_cargo(ax, landX, boxY, 0, boxD, boxW, boxH, [0.85 0.55 0.25]);
    text(ax, landX, boxY, boxH+0.3, 'DELIVERED',...
         'HorizontalAlignment','center','FontWeight','bold',...
         'FontSize',9,'Color',[0 0.6 0],'Tag','cargo');
    pause(0.5);

    % ----- Phase 12 : close ramp and drive away -----
    nF   = round(0.6*fps);
    angs = linspace(90, 0, nF);
    set(ht,'String','Phase 12 : Close Ramp & Depart');
    for i = 1:nF
        delete(findobj(ax,'Tag','robot'));
        draw_robot(ax, R, dropX, 0, angs(i), 0);
        drawnow; pause(dt);
    end
    nF = round(0.8*fps);
    xs = linspace(dropX, dropX - 8, nF);
    for i = 1:nF
        delete(findobj(ax,'Tag','robot'));
        draw_robot(ax, R, xs(i), 0, 0, 0);
        drawnow; pause(dt);
    end

    set(ht,'String','Sequence Complete');
end

%% ================================================================
function R = robot_dims()
% Proportions matched to CAD reference.
% CAD shows:  body height ~2cm, ground clearance ~1cm, wheel dia ~12.5cm
% We scale everything ×1 (use cm as our unit, then the scene is ~25cm wide).
% For better visual we use a ×0.1 m scale so values are convenient.
    R.baseL   = 2.8;       % body length  (X)  — COMPACT
    R.baseW   = 2.6;       % body width   (Y)
    R.baseH   = 1.0;       % body height  (Z)  — thin slab like CAD
    R.baseZ   = 0.5;       % ground clearance to bottom of chassis
    R.wheelR  = 1.2;       % wheel radius — BIG  (diameter ≈ 2× body height)
    R.wheelT  = 0.30;      % tyre thickness
    R.nSpoke  = 6;         % spokes per wheel (CAD shows 6)
    R.rampLen = 2.5;       % ramp length
    R.rampW   = 2.2;       % ramp width
    R.rampThk = 0.08;      % ramp panel thickness
    R.actLen  = 2.2;       % linear actuator max stroke
    R.fingL   = 0.55;      % finger prong length (fixed at ramp tip)
    R.fingGap = 2.2;       % Y gap between finger pair (OPEN, wider than cargo)
    R.actBodyL= 0.5;       % actuator cylinder body length
end

%% ================================================================
function draw_robot(ax, R, cx, cy, gateAng, actExt, fingOpen)
% cx,cy    : centre of chassis
% gateAng  : 0=closed(vertical up), 90=fully open (tip on ground)
% actExt   : 0..1 normalised actuator extension
% fingOpen : 0..1 finger openness (1=wide open, 0=closed/gripping)
    if nargin < 7, fingOpen = 1; end

    bx1 = cx - R.baseL/2;  bx2 = cx + R.baseL/2;
    by1 = cy - R.baseW/2;  by2 = cy + R.baseW/2;
    bz1 = R.baseZ;         bz2 = R.baseZ + R.baseH;

    %% ---- chassis (compact slab) ----
    draw_boxPatch(ax, bx1,by1,bz1, R.baseL,R.baseW,R.baseH,...
                  [0.55 0.60 0.65], 0.50, 'robot');

    %% ---- 4 big wheels (axle along Y — visible as circles from side) ----
    % In the CAD the wheels are much larger than the body.
    % Front pair near front face, rear pair near rear.
    wCZ   = R.wheelR;                           % centre height = radius (sitting on ground)
    ofsX  = R.baseL/2 - 0.15;                   % almost at body edges
    ofsY  = R.baseW/2 + R.wheelT/2 + 0.08;      % just outside the body

    wpos = [ cx+ofsX, cy-ofsY, wCZ;    % front-left
             cx+ofsX, cy+ofsY, wCZ;    % front-right
             cx-ofsX, cy-ofsY, wCZ;    % rear-left
             cx-ofsX, cy+ofsY, wCZ ];  % rear-right

    for w = 1:4
        draw_wheel(ax, wpos(w,:), R.wheelR, R.wheelT, R.nSpoke, 'robot');
    end

    %% ---- axle bars connecting wheels to body ----
    for w = 1:4
        wp = wpos(w,:);
        bodyY = cy + sign(wp(2)-cy)*R.baseW/2;
        % horizontal bar from body side to wheel centre
        plot3(ax,[wp(1) wp(1)],[bodyY wp(2)],[bz1+R.baseH/2 wp(3)],...
              'Color',[0.3 0.3 0.3],'LineWidth',3.5,'Tag','robot');
    end

    %% ---- conveyor belt (dark strip on chassis floor) ----
    cy1 = cy - R.rampW/2;  cy2 = cy + R.rampW/2;
    cz  = bz1 + 0.02;
    fill3(ax,[bx1+0.15 bx2-0.15 bx2-0.15 bx1+0.15],...
             [cy1 cy1 cy2 cy2],[cz cz cz cz],...
         [0.18 0.18 0.18],'FaceAlpha',0.75,'Tag','robot','EdgeColor','k');
    % rollers
    nR = 5;  rxs = linspace(bx1+0.25, bx2-0.25, nR);
    for r = 1:nR
        plot3(ax,[rxs(r) rxs(r)],[cy1 cy2],[cz+0.01 cz+0.01],...
              'Color',[0.5 0.5 0.5],'LineWidth',1.5,'Tag','robot');
    end

    %% ---- gate / ramp (bottom-hinged, inclined when open) ----
    % Hinge at (bx2, *, bz1).
    % Parametric: x(d) = bx2 + d*sin(a), z(d) = bz1 + d*cos(a)
    % At a=0 ramp points UP (closed).  We want tip at z=0 when fully open:
    %   a_max = acos(-baseZ/rampLen)
    a_max    = acos(-R.baseZ / R.rampLen);
    a_actual = (gateAng / 90) * a_max;

    % ramp direction vectors
    dirX = sin(a_actual);   dirZ = cos(a_actual);
    nrmX = -dirZ;           nrmZ = dirX;         % thickness normal

    hngX = bx2;             hngZ = bz1;
    tipX = hngX + R.rampLen * dirX;
    tipZ = hngZ + R.rampLen * dirZ;

    % draw ramp panel (extruded quad)
    ry1 = cy - R.rampW/2;  ry2 = cy + R.rampW/2;
    t   = R.rampThk / 2;
    px  = [hngX-nrmX*t, tipX-nrmX*t, tipX+nrmX*t, hngX+nrmX*t];
    pz  = [hngZ-nrmZ*t, tipZ-nrmZ*t, tipZ+nrmZ*t, hngZ+nrmZ*t];
    % two side faces
    for sideY = [ry1 ry2]
        fill3(ax, px, [sideY sideY sideY sideY], pz,...
              [0.55 0.55 0.55],'FaceAlpha',0.85,'Tag','robot','EdgeColor','k');
    end
    % connecting faces (top, bottom, hinge-edge, tip-edge)
    for k = 1:4
        kn = mod(k,4)+1;
        fill3(ax,[px(k) px(kn) px(kn) px(k)],...
                 [ry1   ry1   ry2   ry2],...
                 [pz(k) pz(kn) pz(kn) pz(k)],...
              [0.50 0.50 0.50],'FaceAlpha',0.85,'Tag','robot','EdgeColor','k');
    end
    % ramp surface highlight (thin line along centre)
    nPts = 10;
    dv   = linspace(0, R.rampLen, nPts);
    lx   = hngX + dv*dirX;  lz = hngZ + dv*dirZ;
    plot3(ax, lx, repmat(cy,1,nPts), lz,...
          'Color',[0.4 0.4 0.4],'LineWidth',1,'Tag','robot');

    %% ---- linear actuator + fingers (through BODY WALLS, independent of ramp) ----
    % The actuator is mounted inside the robot body at MID-HEIGHT of the
    % chassis walls. It extends HORIZONTALLY through the front wall.
    % The fingers are at the rod end — completely independent of ramp.
    actZ   = bz1 + R.baseH / 2;               % MID-HEIGHT of chassis walls
    fingZ  = actZ;                             % fingers at same height as actuator (horizontal)
    stroke = actExt * R.actLen;

    % Two actuator cylinders (one per finger side)
    % fingOpen controls the Y spread: 1 = wide open (wider than cargo),
    %                                  0 = closed (touching cargo sides)
    closedGap = 1.6;                           % matches cargo width — fingers just touch box
    currentGap = closedGap + fingOpen * (R.fingGap - closedGap);
    for sideY = [-1 1] * (currentGap/2)
        % --- cylinder body (inside the chassis, dark) ---
        cylStartX = bx2 - R.baseL * 0.6;      % deep inside the body
        cylEndX   = bx2;                        % at front wall
        plot3(ax,[cylStartX cylEndX],[cy+sideY cy+sideY],[actZ actZ],...
              'Color',[0.25 0.25 0.25],'LineWidth',6,'Tag','robot');

        % --- wall guide / bushing at front wall ---
        draw_boxPatch(ax, bx2-0.06, cy+sideY-0.09, actZ-0.09,...
                      0.12, 0.18, 0.18,...
                      [0.4 0.4 0.4], 0.9, 'robot');

        % --- piston rod (extends horizontally out through front wall) ---
        rodEndX = bx2 + stroke;
        plot3(ax,[cylEndX rodEndX],[cy+sideY cy+sideY],[actZ actZ],...
              'Color',[0.75 0.75 0.15],'LineWidth',3,'Tag','robot');

        % --- finger (straight horizontal prong at rod end) ---
        plot3(ax,[rodEndX rodEndX+R.fingL],[cy+sideY cy+sideY],...
              [actZ actZ],...
              'Color',[0.85 0.15 0.15],'LineWidth',4,'Tag','robot');
        % small inward hook
        hookDirY = -sign(sideY) * 0.18;
        plot3(ax,[rodEndX+R.fingL rodEndX+R.fingL],...
              [cy+sideY cy+sideY+hookDirY],...
              [actZ actZ],...
              'Color',[0.85 0.15 0.15],'LineWidth',4,'Tag','robot');
    end

    % --- crossbar connecting the two rod ends ---
    rodEndX = bx2 + stroke;
    plot3(ax,[rodEndX rodEndX],...
          [cy-currentGap/2 cy+currentGap/2],...
          [actZ actZ],...
          'Color',[0.3 0.3 0.3],'LineWidth',3,'Tag','robot');

    %% ---- rear attachment point (small block at back, like CAD) ----
    rbx = bx1 - 0.25;
    draw_boxPatch(ax, rbx, cy-0.2, bz1+R.baseH*0.2, 0.25, 0.4, R.baseH*0.6,...
                  [0.4 0.4 0.4], 0.9, 'robot');

    %% ---- label ----
    text(ax, cx, cy, bz2+0.4, 'AMR','HorizontalAlignment','center',...
         'FontWeight','bold','FontSize',10,'Color',[0 0.2 0.6],'Tag','robot');
    if gateAng > 5
        text(ax,(hngX+tipX)/2, cy, (hngZ+tipZ)/2-0.35, 'RAMP',...
             'HorizontalAlignment','center','FontSize',8,...
             'Color',[0.35 0.35 0.35],'Tag','robot');
    end
end

%% ================================================================
function draw_wheel(ax, centre, radius, thickness, nSpoke, tag)
% Wheel with axle along Y-axis (visible as a circle from side view).
% Uses a cylinder rotated so that the axle is along Y.
    n = 48;  % resolution
    [cylZ, cylX, cylY] = cylinder(1, n);   % default: axle along Z
    % We need axle along Y → rotate: Y←Z, Z←X, X←Y  (or just remap)
    % Actually: cylinder gives axle along Z.
    % To get axle along Y, swap: Xout=cylX, Yout=cylZ, Zout=cylY
    % Then scale.
    Xout = cylX * radius + centre(1);       % ring in X-Z plane
    Yout = cylZ * thickness - thickness/2 + centre(2);  % extrude along Y
    Zout = cylY * radius + centre(3);

    % Tyre surface
    surf(ax, Xout, Yout, Zout,...
         'FaceColor',[0.12 0.12 0.12],'EdgeColor','none',...
         'FaceAlpha',0.92,'Tag',tag,'FaceLighting','gouraud');

    % Hub caps on each side
    th = linspace(0, 2*pi, n+1);
    hubR = radius * 0.55;
    for side = [-1 1]
        hy = centre(2) + side * thickness/2;
        hx = centre(1) + hubR * cos(th);
        hz = centre(3) + hubR * sin(th);
        fill3(ax, hx, repmat(hy,size(th)), hz,...
              [0.45 0.45 0.45],'FaceAlpha',0.95,'Tag',tag,...
              'EdgeColor',[0.35 0.35 0.35],'FaceLighting','gouraud');
    end

    % Spokes (from centre to near rim, visible through hub cap)
    for s = 1:nSpoke
        a = 2*pi*(s-1)/nSpoke;
        sx = [centre(1), centre(1) + radius*0.88*cos(a)];
        sz = [centre(3), centre(3) + radius*0.88*sin(a)];
        plot3(ax, sx, [centre(2) centre(2)], sz,...
              'Color',[0.5 0.5 0.5],'LineWidth',2,'Tag',tag);
    end

    % Rim ring (circle at radius)
    rx = centre(1) + radius*cos(th);
    rz = centre(3) + radius*sin(th);
    plot3(ax, rx, repmat(centre(2),size(th)), rz,...
          'Color',[0.3 0.3 0.3],'LineWidth',1.5,'Tag',tag);
end

%% ================================================================
function draw_boxPatch(ax, x,y,z, dx,dy,dz, col, alph, tag)
    verts = [x x+dx x+dx x   x   x+dx x+dx x   ;
             y y    y+dy y+dy y   y    y+dy y+dy ;
             z z    z    z    z+dz z+dz z+dz z+dz]';
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 4 8 5; 2 3 7 6];
    patch(ax,'Vertices',verts,'Faces',faces,...
          'FaceColor',col,'FaceAlpha',alph,'EdgeColor','k',...
          'Tag',tag,'FaceLighting','gouraud');
end

%% ================================================================
function draw_cargo(ax, bx, by, bz, bd, bw, bh, col)
    draw_boxPatch(ax, bx-bd/2, by-bw/2, bz, bd, bw, bh, col, 0.85, 'cargo');
end
