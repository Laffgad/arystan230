function Env = buildEnvManual()
    Env.bounds = [-0.25 25.25; -20.75 20.75];
    
    obs = repmat(struct( ...
        "poly", polyshape(), ...
        "faceColor", [0 0 0], ...
        "faceAlpha", 1.0, ...
        "edgeColor", [0 0 0], ...
        "edgeWidth", 1.0), 0, 1);
    
    blk = [0 0 0];
    
    % Outer Walls
    obs(end+1) = makeRectObs(0, -3.75, 12, 0.25, blk, 1.0, blk, 1.0);
    obs(end+1) = makeRectObs(0, 3.5, 12, 0.25, blk, 1.0, blk, 1.0);
    obs(end+1) = makeRectObs(14, -3.0, 4.25, 0.25, blk, 1.0, blk, 1.0);
    obs(end+1) = makeRectObs(14, 2.75, 4.25, 0.25, blk, 1.0, blk, 1.0);
    obs(end+1) = makeRectObs(0, -3.75, 0.25, 7.25, blk, 1.0, blk, 1.0);
    obs(end+1) = makeRectObs(19.75, -1.25, 0.25, 2.5, blk, 1.0, blk, 1.0);
    
    % Internal Obstacles
    obs(end+1) = makeRectObs(18.0, 1.0, 1.75, 0.25, blk, 1.0, blk, 1.0);
    obs(end+1) = makeRectObs(18.0, -1.0, 1.75, -0.25, blk, 1.0, blk, 1.0);
    obs(end+1) = makeRectObs(19.0, -0.05, 0.75, 0.1, blk, 1.0, blk, 1.0);
    
    obs(end+1) = makeRectObs(18.0, 1.0, 0.25, 1.75, blk, 1.0, blk, 1.0);
    obs(end+1) = makeRectObs(15.75, 1.0, 0.25, 1.75, blk, 1.0, blk, 1.0);
    % obs(end+1) = makeRectObs(15.75, 2.75, 2.5, 0.25, blk, 1.0, blk, 1.0);
    obs(end+1) = makeRectObs(16.95, 2.0, 0.1, 0.75, blk, 1.0, blk, 1.0);
    
    obs(end+1) = makeRectObs(18.0, -1.0, 0.25, -1.75, blk, 1.0, blk, 1.0);
    obs(end+1) = makeRectObs(15.75, -1.0, 0.25, -1.75, blk, 1.0, blk, 1.0);
    % obs(end+1) = makeRectObs(15.75, -2.75, 2.5, -0.25, blk, 1.0, blk, 1.0);
    obs(end+1) = makeRectObs(16.95, -2.0, 0.1, -0.75, blk, 1.0, blk, 1.0);
    
    obs(end+1) = makeRectObs(14.0, 1.0, 1.75, 0.25, blk, 1.0, blk, 1.0);
    obs(end+1) = makeRectObs(14.0, -1.0, 1.75, -0.25, blk, 1.0, blk, 1.0);
    obs(end+1) = makeRectObs(14.0, 1.0, 0.25, 1.75, blk, 1.0, blk, 1.0);
    obs(end+1) = makeRectObs(14.0, -1.0, 0.25, -1.75, blk, 1.0, blk, 1.0);

    obs(end+1) = makeRectObs(10, 2.0, 1.75, 0.25, blk, 1.0, blk, 1.0);
    obs(end+1) = makeRectObs(10, -2.0, 1.75, -0.25, blk, 1.0, blk, 1.0);
    obs(end+1) = makeRectObs(11.75, 2.0, 0.25, 1.5, blk, 1.0, blk, 1.0);
    obs(end+1) = makeRectObs(11.75, -2.0, 0.25, -1.5, blk, 1.0, blk, 1.0);
    
    % Circles
    obs(end+1) = makeCircleObs(7, 1.5, 0.5, 80, blk, 1.0, blk, 1.0);
    obs(end+1) = makeCircleObs(7, -2.5, 1, 80, blk, 1.0, blk, 1.0);
    % obs(end+1) = makeCircleObs(4, 1, 0.5, 80, blk, 1.0, blk, 1.0);
    % obs(end+1) = makeCircleObs(4, -1, 0.5, 80, blk, 1.0, blk, 1.0);
    
    Env.obstacles = obs;
end

function o = makeRectObs(x, y, w, h, faceColor, faceAlpha, edgeColor, edgeWidth)
    o = struct( ...
        "poly", rectPoly(x, y, w, h), ...
        "faceColor", [0 0 0], ...
        "faceAlpha", 1.0, ...
        "edgeColor", [0 0 0], ...
        "edgeWidth", 1.0);
    
    if nargin >= 5 && ~isempty(faceColor), o.faceColor = faceColor; end
    if nargin >= 6 && ~isempty(faceAlpha), o.faceAlpha = faceAlpha; end
    if nargin >= 7 && ~isempty(edgeColor), o.edgeColor = edgeColor; end
    if nargin >= 8 && ~isempty(edgeWidth), o.edgeWidth = edgeWidth; end
end

function o = makeCircleObs(cx, cy, r, n, faceColor, faceAlpha, edgeColor, edgeWidth)
    o = struct( ...
        "poly", circlePoly(cx, cy, r, n), ...
        "faceColor", [0 0 0], ...
        "faceAlpha", 1.0, ...
        "edgeColor", [0 0 0], ...
        "edgeWidth", 1.0);
    
    if nargin >= 5 && ~isempty(faceColor), o.faceColor = faceColor; end
    if nargin >= 6 && ~isempty(faceAlpha), o.faceAlpha = faceAlpha; end
    if nargin >= 7 && ~isempty(edgeColor), o.edgeColor = edgeColor; end
    if nargin >= 8 && ~isempty(edgeWidth), o.edgeWidth = edgeWidth; end
end