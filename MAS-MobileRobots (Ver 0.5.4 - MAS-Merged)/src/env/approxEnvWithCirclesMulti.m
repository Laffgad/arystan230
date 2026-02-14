function [EnvC, circles] = approxEnvWithCirclesMulti(Env, extraMargin, spacingF)
    if nargin < 2 || isempty(extraMargin), extraMargin = 0.0; end

    nObs = numel(Env.obstacles);

    obsOut = repmat(struct( ...
        "poly", polyshape(), ...
        "faceColor", [0 0 0], ...
        "faceAlpha", 1.0, ...
        "edgeColor", [0 0 0], ...
        "edgeWidth", 1.0), 0, 1);

    circles = repmat(struct('c',[0 0],'r',0,'srcIdx',0), 0, 1);

    for i = 1:nObs
        pg = Env.obstacles(i).poly;
        if isempty(pg) || (isa(pg,'polyshape') && pg.NumRegions == 0)
            continue;
        end

        old = Env.obstacles(i);

        [x, y] = boundary(pg);
        m = ~(isnan(x) | isnan(y));
        x = x(m); y = y(m);

        if isempty(x)
            V = pg.Vertices;
            x = V(:,1); y = V(:,2);
            if isempty(x), continue; end
        end

        c0 = [mean(x) mean(y)];
        d0 = hypot(x - c0(1), y - c0(2));
        rMean = mean(d0);
        if rMean > 0 && std(d0)/rMean < 0.03
            rBase = max(d0);
            rInfl = rBase + extraMargin;
            obsOut(end+1) = makeCircleObs_local(c0(1), c0(2), rInfl, 80, old.faceColor, old.faceAlpha, old.edgeColor, old.edgeWidth);
            circles(end+1) = struct('c', c0, 'r', rInfl, 'srcIdx', i);
            continue;
        end

        xmin = min(x); xmax = max(x);
        ymin = min(y); ymax = max(y);

        w = xmax - xmin;
        h = ymax - ymin;

        rBase = 0.5 * min(w, h);
        if rBase <= 0, continue; end

        rInfl = rBase + extraMargin;

        sMajor = 2*rBase*spacingF;
        sMinor = 2*rBase*spacingF;

        if w >= h
            xs = centersLine(xmin, xmax, rBase, sMajor, false);
            ys = centersLine(ymin, ymax, rBase, sMinor, true);
        else
            ys = centersLine(ymin, ymax, rBase, sMajor, false);
            xs = centersLine(xmin, xmax, rBase, sMinor, true);
        end

        for a = 1:numel(xs)
            for b = 1:numel(ys)
                obsOut(end+1) = makeCircleObs_local(xs(a), ys(b), rInfl, 60, old.faceColor, old.faceAlpha, old.edgeColor, old.edgeWidth);
                circles(end+1) = struct('c',[xs(a) ys(b)], 'r', rInfl, 'srcIdx', i);
            end
        end
    end

    EnvC = Env;
    EnvC.obstacles = obsOut;
end

function v = centersLine(a, b, rBase, s, forceOdd)
    if b < a, t = a; a = b; b = t; end
    L = b - a;
    if L <= 2*rBase
        v = (a+b)/2;
        return;
    end
    span = L - 2*rBase;
    n = max(1, floor(span / s) + 1);
    if forceOdd && mod(n,2)==0
        n = n + 1;
    end
    if n == 1
        v = (a+b)/2;
    else
        v = linspace(a + rBase, b - rBase, n);
    end
end

function o = makeCircleObs_local(cx, cy, r, n, faceColor, faceAlpha, edgeColor, edgeWidth)
    o = struct( ...
        "poly", circlePoly_local(cx, cy, r, n), ...
        "faceColor", [0 0 0], ...
        "faceAlpha", 1.0, ...
        "edgeColor", [0 0 0], ...
        "edgeWidth", 1.0);

    if nargin >= 5 && ~isempty(faceColor), o.faceColor = faceColor; end
    if nargin >= 6 && ~isempty(faceAlpha), o.faceAlpha = faceAlpha; end
    if nargin >= 7 && ~isempty(edgeColor), o.edgeColor = edgeColor; end
    if nargin >= 8 && ~isempty(edgeWidth), o.edgeWidth = edgeWidth; end
end

function p = circlePoly_local(cx, cy, r, n)
    if nargin < 4 || isempty(n), n = 64; end
    th = linspace(0, 2*pi, n+1);
    th(end) = [];
    x = cx + r*cos(th);
    y = cy + r*sin(th);
    p = polyshape(x, y);
end
