function CS = buildCSpaceGridXY(Env, P)
    CS.dx  = 1/P.res;

    xmin = Env.bounds(1,1); xmax = Env.bounds(1,2);
    ymin = Env.bounds(2,1); ymax = Env.bounds(2,2);

    CS.xs = (xmin + CS.dx/2) : CS.dx : (xmax - CS.dx/2);
    CS.ys = (ymin + CS.dx/2) : CS.dx : (ymax - CS.dx/2);

    [X, Y] = meshgrid(CS.xs, CS.ys);
    ptsX = X(:); ptsY = Y(:);

    inflated = repmat(polyshape(), numel(Env.obstacles), 1);
    for i = 1:numel(Env.obstacles)
        inflated(i) = polybuffer(Env.obstacles(i).poly, P.robotRadius+0.1);
    end

    occ = false(numel(ptsX), 1);
    for i = 1:numel(inflated)
        occ = occ | isinterior(inflated(i), ptsX, ptsY);
    end

    CS.occ = reshape(occ, size(X));

    CS.inflatedObstacles = inflated;
end