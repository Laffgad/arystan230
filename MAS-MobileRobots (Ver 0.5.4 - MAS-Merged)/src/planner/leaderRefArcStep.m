function [xr, yr, thr, vr, wr, st] = leaderRefArcStep(pathXY, leaderPos, vCmd, st, P)

dt = P.dt;

xy = pathXY(:,1:2);
if size(xy,1) < 2
    xr = xy(1,1); yr = xy(1,2);
    thr = 0; vr = 0; wr = 0;
    st = struct('s',0,'thPrev',thr);
    return;
end

dseg = hypot(diff(xy(:,1)), diff(xy(:,2)));
S = [0; cumsum(dseg)];
Send = S(end);

if isempty(st) || ~isfield(st,'s') || ~isfinite(st.s) || ~isfield(st,'thPrev') || ~isfinite(st.thPrev)
    [s0, th0] = projectToPolylineArc(xy, S, leaderPos);
    st = struct('s', s0, 'thPrev', th0);
end

st.s = clamp(st.s + max(vCmd,0)*dt, 0, Send);

xr = interp1(S, xy(:,1), st.s, 'linear', 'extrap');
yr = interp1(S, xy(:,2), st.s, 'linear', 'extrap');

lookahead = max(P.robotRadius, 0.05);
s2 = clamp(st.s + lookahead, 0, Send);

x2 = interp1(S, xy(:,1), s2, 'linear', 'extrap');
y2 = interp1(S, xy(:,2), s2, 'linear', 'extrap');

dx = x2 - xr; dy = y2 - yr;
if hypot(dx,dy) < 1e-9
    thr = st.thPrev;
else
    thr = atan2(dy, dx);
end

vr = vCmd;

wr = wrapToPiCustom(thr - st.thPrev) / dt;
wr = clamp(wr, -P.wMax, P.wMax);
st.thPrev = thr;

end

function [sBest, thBest] = projectToPolylineArc(xy, S, p)
px = p(1); py = p(2);

bestD2 = inf;
sBest  = 0;

vx0 = xy(2,1) - xy(1,1);
vy0 = xy(2,2) - xy(1,2);
if hypot(vx0,vy0) < 1e-12
    thBest = 0;
else
    thBest = atan2(vy0, vx0);
end

for i = 1:size(xy,1)-1
    ax = xy(i,1);  ay = xy(i,2);
    bx = xy(i+1,1);by = xy(i+1,2);

    vx = bx-ax; vy = by-ay;
    L2 = vx*vx + vy*vy;

    if L2 < 1e-12
        t = 0;
        segL = 0;
    else
        t = ((px-ax)*vx + (py-ay)*vy) / L2;
        t = clamp(t, 0, 1);
        segL = sqrt(L2);
    end

    qx = ax + t*vx;
    qy = ay + t*vy;

    d2 = (px-qx)^2 + (py-qy)^2;
    if d2 < bestD2
        bestD2 = d2;
        sBest  = S(i) + t*segL;
        if segL < 1e-12
            % keep previous thBest
        else
            thBest = atan2(vy, vx);
        end
    end
end
end