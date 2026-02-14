function pathXY = astarGridXY(CS, xStart, xGoal, conn)
    if nargin < 4 || isempty(conn), conn = 8; end
    if conn ~= 4 && conn ~= 8, error('conn must be 4 or 8'); end

    occ = CS.occ;
    [H, W] = size(occ);

    s = worldToGrid(CS, xStart(1), xStart(2));
    g = worldToGrid(CS, xGoal(1),  xGoal(2));

    if ~inBounds(s, H, W) || ~inBounds(g, H, W)
        error('Start/goal outside grid bounds.');
    end
    if occ(s(1), s(2)), error('Start is in obstacle (occupied cell).'); end
    if occ(g(1), g(2)), error('Goal is in obstacle (occupied cell).'); end

    INF = 1e18;
    gScore = INF * ones(H, W);
    fScore = INF * ones(H, W);
    cameFrom = zeros(H, W, 2, 'int32');
    inOpen = false(H, W);
    inClosed = false(H, W);

    gScore(s(1), s(2)) = 0;
    fScore(s(1), s(2)) = heuristic(s, g, CS.dx);

    openI = zeros(H*W, 1, 'int32');
    openJ = zeros(H*W, 1, 'int32');
    openN = 1;
    openI(1) = s(1); openJ(1) = s(2);
    inOpen(s(1), s(2)) = true;

    if conn == 4
        moves = int32([ -1  0;
                         1  0;
                         0 -1;
                         0  1 ]);
        moveCost = [1;1;1;1] * CS.dx;
    else
        moves = int32([ -1  0;
                         1  0;
                         0 -1;
                         0  1;
                        -1 -1;
                        -1  1;
                         1 -1;
                         1  1 ]);
        moveCost = [1;1;1;1;sqrt(2);sqrt(2);sqrt(2);sqrt(2)]' * CS.dx;
    end

    while openN > 0
        bestIdx = 1;
        bi = openI(1); bj = openJ(1);
        bestF = fScore(bi, bj);
        for t = 2:openN
            ii = openI(t); jj = openJ(t);
            ft = fScore(ii, jj);
            if ft < bestF
                bestF = ft;
                bestIdx = t;
                bi = ii; bj = jj;
            end
        end

        current = [bi bj];

        openI(bestIdx) = openI(openN);
        openJ(bestIdx) = openJ(openN);
        openN = openN - 1;
        inOpen(current(1), current(2)) = false;

        if current(1) == g(1) && current(2) == g(2)
            pathXY = reconstructPath(CS, cameFrom, s, g);
            return;
        end

        inClosed(current(1), current(2)) = true;

        for m = 1:size(moves,1)
            ni = current(1) + moves(m,1);
            nj = current(2) + moves(m,2);

            if ~inBounds([ni nj], H, W), continue; end
            if inClosed(ni, nj), continue; end
            if occ(ni, nj), continue; end

            if conn == 8
                di = moves(m,1); dj = moves(m,2);
                if abs(di) == 1 && abs(dj) == 1
                    if occ(current(1)+di, current(2)) || occ(current(1), current(2)+dj)
                        continue;
                    end
                end
            end

            tentativeG = gScore(current(1), current(2)) + moveCost(m);

            if tentativeG < gScore(ni, nj)
                cameFrom(ni, nj, :) = int32(current);
                gScore(ni, nj) = tentativeG;
                fScore(ni, nj) = tentativeG + heuristic([ni nj], g, CS.dx);

                if ~inOpen(ni, nj)
                    openN = openN + 1;
                    openI(openN) = ni;
                    openJ(openN) = nj;
                    inOpen(ni, nj) = true;
                end
            end
        end
    end

    pathXY = [];
end

function h = heuristic(a, b, dx)
    da = double(a - b);
    h = dx * hypot(da(1), da(2));
end

function tf = inBounds(idx, H, W)
    tf = idx(1) >= 1 && idx(1) <= H && idx(2) >= 1 && idx(2) <= W;
end

function ij = worldToGrid(CS, x, y)
    j = int32(round((x - CS.xs(1))/CS.dx) + 1);
    i = int32(round((y - CS.ys(1))/CS.dx) + 1);
    ij = [i j];
end

function pathXY = reconstructPath(CS, cameFrom, s, g)
    maxLen = numel(CS.occ);
    pi = zeros(maxLen,1,'int32');
    pj = zeros(maxLen,1,'int32');
    n = 1;
    pi(n)=g(1); pj(n)=g(2);

    cur = g;
    while ~(cur(1)==s(1) && cur(2)==s(2))
        prev = squeeze(cameFrom(cur(1), cur(2), :))';
        if all(prev == 0)
            pathXY = [];
            return;
        end
        n = n + 1;
        pi(n)=prev(1); pj(n)=prev(2);
        cur = prev;
    end

    pi = flipud(pi(1:n));
    pj = flipud(pj(1:n));

    xs = CS.xs(double(pj));
    ys = CS.ys(double(pi));
    pathXY = [xs(:), ys(:)];
end
