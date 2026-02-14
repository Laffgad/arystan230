function [Xref, Uref, t] = pathToUnicycleRef(Pxy, dt, vMax, wMax, thetaGoal, opts)
if nargin < 6 || isempty(opts), opts = struct(); end
if ~isfield(opts,'stopDist'), opts.stopDist = 1.0; end
if ~isfield(opts,'thetaBlendDist'), opts.thetaBlendDist = 0.0; end
if ~isfield(opts,'finalSpin'), opts.finalSpin = true; end
% New option: Start Theta for initial alignment
if ~isfield(opts,'startTheta'), opts.startTheta = NaN; end

Pxy = uniqueConsecutive(Pxy);
if size(Pxy,1) < 2
    Xref = [Pxy(1,1) Pxy(1,2) thetaGoal];
    Uref = [0 0];
    t = 0;
    return;
end

% --- Standard Path Generation ---
ds = sqrt(sum(diff(Pxy,1,1).^2,2));
s = [0; cumsum(ds)];
L = s(end);

x = Pxy(:,1); y = Pxy(:,2);

dx = gradient(x, s);
dy = gradient(y, s);
ddx = gradient(dx, s);
ddy = gradient(dy, s);

theta = atan2(dy, dx);
theta = unwrap(theta);

den = (dx.^2 + dy.^2).^(3/2);
kappa = (dx.*ddy - dy.*ddx) ./ max(den, 1e-9);

vCap = wMax ./ max(abs(kappa), 1e-6);
vS = min(vMax, vCap);

if opts.stopDist > 0
    u = min(1, max(0, (L - s) / opts.stopDist));
    scale = u.^2 .* (3 - 2*u);
    vS = vS .* scale;
end

sGrid = 0;
tGrid = 0;

while sGrid(end) < L
    sNow = sGrid(end);
    vNow = interp1(s, vS, sNow, 'linear', 'extrap');
    dsStep = max(1e-6, vNow * dt);
    sNext = min(L, sNow + dsStep);
    sGrid(end+1,1) = sNext;
    tGrid(end+1,1) = tGrid(end) + dt;
    if L - sNext < 1e-9
        break;
    end
end

xg = interp1(s, x, sGrid, 'pchip');
yg = interp1(s, y, sGrid, 'pchip');
thg = interp1(s, theta, sGrid, 'pchip');
kg = interp1(s, kappa, sGrid, 'pchip');

vg = interp1(s, vS, sGrid, 'linear', 'extrap');
wg = vg .* kg;
wg = min(max(wg, -wMax), wMax);

if opts.thetaBlendDist > 0
    thEnd = thg(end);
    thGoalAdj = thetaGoal + 2*pi*round((thEnd - thetaGoal)/(2*pi));
    a = min(1, max(0, (sGrid - (L - opts.thetaBlendDist)) / opts.thetaBlendDist));
    a = a.^2 .* (3 - 2*a);
    thg = (1-a).*thg + a.*thGoalAdj;
end

thg = wrapPi(thg);

vg(end) = 0;
wg(end) = 0;

Xref = [xg yg thg];
Uref = [vg wg];
t = tGrid;

% --- PREPEND INITIAL SPIN (New Logic) ---
if ~isnan(opts.startTheta)
    thStartPath = thg(1); 
    thCurrent   = opts.startTheta;
    
    dthStart = wrapPi(thStartPath - thCurrent);
    
    % If misalignment is significant, spin in place first
    if abs(dthStart) > 1e-3
        % Nspin is number of intervals (steps)
        Nspin = max(1, ceil(abs(dthStart) / (wMax * dt)));
        wSpin = dthStart / (Nspin * dt);
        wSpin = max(-wMax, min(wMax, wSpin)); 
        
        % Generate spin trajectory (t=0 to t=Tspin)
        tSpin = (0:Nspin)' * dt; 
        thSpin = thCurrent + tSpin * wSpin; 
        thSpin = wrapPi(thSpin);
        
        xSpin = repmat(xg(1), Nspin+1, 1);
        ySpin = repmat(yg(1), Nspin+1, 1);
        
        Xspin = [xSpin, ySpin, thSpin];
        Uspin = [zeros(Nspin+1,1), repmat(wSpin, Nspin+1, 1)];
        
        % Splice: [Spin Traj] + [Path Traj (shifted)]
        % Remove the first point of the original path (t=0) to avoid duplicate state
        Xref = [Xspin; Xref(2:end,:)];
        Uref = [Uspin; Uref(2:end,:)];
        
        tShifted = tGrid(2:end) + tSpin(end);
        t = [tSpin; tShifted];
    end
end

% --- APPEND FINAL SPIN (Existing Logic) ---
if opts.finalSpin
    th0 = Xref(end,3);
    dth = wrapPi(thetaGoal - th0);
    if abs(dth) > 1e-6
        N = max(1, ceil(abs(dth) / (wMax*dt)));
        w = dth / (N*dt);
        w = min(max(w, -wMax), wMax);
        thAdd = th0 + (1:N)' * w * dt;
        thAdd = wrapPi(thAdd);

        xAdd = repmat(Xref(end,1), N, 1);
        yAdd = repmat(Xref(end,2), N, 1);

        Xref = [Xref; [xAdd yAdd thAdd]];
        Uref = [Uref; [zeros(N,1) w*ones(N,1)]];
        if isempty(t)
             t = (1:N)'*dt;
        else
             t = [t; (t(end) + (1:N)'*dt)];
        end
        Uref(end,:) = [0 0];
        Xref(end,3) = wrapPi(thetaGoal);
    else
        Xref(end,3) = wrapPi(thetaGoal);
    end
end
end

function P = uniqueConsecutive(P)
keep = [true; any(diff(P,1,1)~=0,2)];
P = P(keep,:);
end

function a = wrapPi(a)
a = atan2(sin(a), cos(a));
end