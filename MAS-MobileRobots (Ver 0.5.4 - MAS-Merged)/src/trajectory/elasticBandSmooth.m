function [P, info] = elasticBandSmooth(Pin, circles, opts, CS)
if nargin < 2, circles = []; end
if nargin < 3 || isempty(opts), opts = struct(); end
if nargin < 4, CS = []; end

opts = fillDefaults(opts);

Pin = uniqueConsecutive(Pin);
if size(Pin,1) < 3
    P = Pin;
    info = struct('iters',0,'meanMove',0,'ok',true);
    return;
end

P = resamplePolyline(Pin, opts.ds);
Pref = P;

[Cc, Cr] = packCircles(circles);

isOccMap = isa(CS,'binaryOccupancyMap') || isa(CS,'occupancyMap');

n = size(P,1);
ok = true;
meanMove = 0;

for it = 1:opts.maxIter
    Pold = P;

    for i = 2:n-1
        p = P(i,:);
        fs = opts.kSmooth * (P(i-1,:) + P(i+1,:) - 2*p);
        fa = opts.kAnchor * (Pref(i,:) - p);

        fo = [0 0];
        if ~isempty(Cc)
            fo = repulsiveForce(p, Cc, Cr, opts);
        end

        dp = opts.step * (fs + fa + fo);
        m = hypot(dp(1), dp(2));
        if m > opts.maxMove
            dp = dp * (opts.maxMove / m);
        end

        pn = p + dp;

        if ~isempty(Cc)
            [pn, inside] = projectIfInside(pn, Cc, Cr);
            if inside
                ok = false;
            end
        end

        if isOccMap
            if getOccupancy(CS, pn) >= 0.5
                pn = p;
                ok = false;
            end
        end

        P(i,:) = pn;
    end

    d = sqrt(sum((P - Pold).^2, 2));
    meanMove = mean(d);
    if meanMove < opts.tol
        break;
    end
end

P = uniqueConsecutive(P);
info = struct('iters', it, 'meanMove', meanMove, 'ok', ok);
end

function opts = fillDefaults(opts)
if ~isfield(opts,'ds'),        opts.ds = 0.2; end
if ~isfield(opts,'maxIter'),   opts.maxIter = 250; end
if ~isfield(opts,'step'),      opts.step = 0.1; end
if ~isfield(opts,'kSmooth'),   opts.kSmooth = 2.0; end
if ~isfield(opts,'kAnchor'),   opts.kAnchor = 0.3; end
if ~isfield(opts,'kObs'),      opts.kObs = 1.0; end
if ~isfield(opts,'kInside'),   opts.kInside = 8.0; end
if ~isfield(opts,'influence'), opts.influence = 0.8; end
if ~isfield(opts,'maxMove'),   opts.maxMove = 0.12; end
if ~isfield(opts,'tol'),       opts.tol = 1e-4; end
end

function P = uniqueConsecutive(P)
if isempty(P), return; end
keep = [true; any(diff(P,1,1)~=0,2)];
P = P(keep,:);
end

function Pout = resamplePolyline(P, ds)
if size(P,1) < 2
    Pout = P;
    return;
end

seg = diff(P,1,1);
L = sqrt(sum(seg.^2,2));
s = [0; cumsum(L)];
S = s(end);

if S <= ds
    Pout = P;
    return;
end

sq = (0:ds:S).';
if sq(end) < S
    sq(end+1,1) = S;
end

xq = interp1(s, P(:,1), sq, 'linear');
yq = interp1(s, P(:,2), sq, 'linear');

Pout = [xq yq];
Pout = uniqueConsecutive(Pout);
end

function [Cc, Cr] = packCircles(circles)
if isempty(circles)
    Cc = zeros(0,2);
    Cr = zeros(0,1);
    return;
end
Cc = reshape([circles.c], 2, []).';
Cr = [circles.r].';
end

function f = repulsiveForce(p, Cc, Cr, opts)
dx = p(1) - Cc(:,1);
dy = p(2) - Cc(:,2);
d  = hypot(dx, dy);

epsd = 1e-9;
dir = [dx dy] ./ max(d, epsd);

clr = d - Cr;

mask = clr < opts.influence;
if ~any(mask)
    f = [0 0];
    return;
end

clrM = clr(mask);
dirM = dir(mask,:);

epsc = 1e-3;
clrEff = max(clrM, epsc);

mag = opts.kObs * (1./clrEff - 1/opts.influence) ./ (clrEff.^2);
inside = clrM <= 0;
if any(inside)
    mag(inside) = opts.kInside * (1./clrEff(inside)) .* (1 + (-clrM(inside)));
end

f = sum(dirM .* mag, 1);
end

function [p, inside] = projectIfInside(p, Cc, Cr)
dx = p(1) - Cc(:,1);
dy = p(2) - Cc(:,2);
d  = hypot(dx, dy);
clr = d - Cr;

[mc, j] = min(clr);
inside = mc <= 0;

if ~inside
    return;
end

c = Cc(j,:);
r = Cr(j);

v = p - c;
nv = hypot(v(1), v(2));
if nv < 1e-9
    v = [1 0];
    nv = 1;
end
p = c + (r + 1e-3) * (v / nv);
end