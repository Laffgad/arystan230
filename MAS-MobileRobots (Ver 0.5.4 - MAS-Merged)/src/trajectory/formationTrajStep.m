function [XrOut, UrOut, state, dbg] = formationTrajStep(posAll, thAll, leaderIdx, XrPlan, UrPlan, envXY, envR, state, P)

posAll = force2xN(posAll);
thAll  = thAll(:);
N      = size(posAll,2);

need(P,'dt'); need(P,'robotRadius'); need(P,'commR'); need(P,'vMax');
need(P,'form'); need(P.form,'R'); need(P.form,'Rdot'); need(P.form,'w0'); need(P.form,'dir');
need(P.form,'arcSpacing'); need(P.form,'minW'); need(P.form,'trackGain'); need(P.form,'slowP');
need(P,'avoid'); need(P.avoid,'clearHard'); need(P.avoid,'clearStart');
need(P,'phase'); need(P.phase,'k'); need(P.phase,'uMax'); need(P.phase,'epsPhi'); need(P.phase,'wMin');

dt   = P.dt;
rBot = P.robotRadius;

commR      = P.commR;
Rk         = P.form.R;
Rdot       = P.form.Rdot;
w0         = P.form.w0;
dir        = P.form.dir;
arcSpacing = P.form.arcSpacing;
minW       = P.form.minW;
trackGain  = P.form.trackGain;
slowP      = P.form.slowP;

clearHard  = P.avoid.clearHard;
clearStart = P.avoid.clearStart;

phaseK    = P.phase.k;
phaseUMax = P.phase.uMax;
epsPhi    = P.phase.epsPhi;
wMinPhase = P.phase.wMin;

if nargin < 8 || isempty(state), state = struct(); end
if ~isfield(state,'phiRef')  || numel(state.phiRef)  ~= N, state.phiRef  = NaN(N,1); end
if ~isfield(state,'thdPrev') || numel(state.thdPrev) ~= N, state.thdPrev = thAll;    end

phiRef  = state.phiRef(:);
thdPrev = state.thdPrev(:);

XrOut = XrPlan;
UrOut = UrPlan;

if ~isfinite(leaderIdx) || leaderIdx < 1 || leaderIdx > N
    state.phiRef(:) = NaN;
    state.thdPrev   = thAll;
    if nargout >= 4, dbg = struct('active',false(N,1),'inComm',false(N,1),'Rk',Rk); else, dbg = []; end
    return;
end

L = posAll(:,leaderIdx);

dL = hypot(posAll(1,:)-L(1), posAll(2,:)-L(2)).';
inComm = (dL <= commR);
inComm(leaderIdx) = false;

phiRef(~inComm)  = NaN;
thdPrev(~inComm) = thAll(~inComm);

newMask = inComm & isnan(phiRef);
if any(newMask)
    phiRef(newMask) = wrapToPiCustom(atan2(posAll(2,newMask)-L(2), posAll(1,newMask)-L(1)));
end

active = inComm & ~isnan(phiRef);
idxA   = find(active);

dPhiDes  = arcSpacing / max(Rk,1e-6);
wMaxCirc = P.vMax / max(Rk,1e-6);
wMinUse  = max(minW, wMinPhase);

wBase = w0*ones(N,1);
if numel(idxA) >= 2
    wBase(idxA) = phaseSpeedCoordinator(phiRef(idxA), w0, dPhiDes, phaseK, phaseUMax, epsPhi, wMinUse, wMaxCirc);
else
    wBase(idxA) = clamp(w0, wMinUse, wMaxCirc);
end

vLead = UrPlan(leaderIdx,1);
thLeadRef = XrPlan(leaderIdx,3);
leadVelW  = vLead * [cos(thLeadRef); sin(thLeadRef)];

wdEff = zeros(N,1);
sRef  = ones(N,1);

for i = idxA(:).'
    phi = phiRef(i);

    uRad = [cos(phi); sin(phi)];
    dirTan = phi + dir*pi/2;
    uTan   = [cos(dirTan); sin(dirTan)];

    xd = L(1) + Rk*uRad(1);
    yd = L(2) + Rk*uRad(2);

    dClear = minClearanceEnv(xd, yd, envXY, envR, rBot);
    sRef(i) = refSpeedScale(dClear, clearHard, clearStart, slowP);

    distErr = hypot(xd - posAll(1,i), yd - posAll(2,i));
    sTrack  = 1.0 / (1.0 + trackGain*distErr);

    wd0 = clamp(wBase(i)*sRef(i), wMinUse, wMaxCirc);
    wd  = wd0 * sTrack;
    wdEff(i) = wd;

    eW = [xd; yd] - posAll(:,i);
    vRefVec = leadVelW + (Rk * wd * dir * uTan) + (Rdot * uRad) + (trackGain * eW);

    vmag = norm(vRefVec);
    if vmag < 1e-9
        vd  = 0.0;
        thd = thAll(i);
        wd_ff = 0.0;
    else
        thd = atan2(vRefVec(2), vRefVec(1));
        vd  = min(vmag, P.vMax);
        wd_ff = wrapToPiCustom(thd - thdPrev(i)) / dt;
    end

    thdPrev(i) = thd;

    XrOut(i,:) = [xd, yd, thd];
    UrOut(i,:) = [vd, wd_ff];
end

for i = idxA(:).'
    phiRef(i) = wrapToPiCustom(phiRef(i) + dt*dir*wdEff(i));
end

state.phiRef  = phiRef;
state.thdPrev = thdPrev;

if nargout >= 4
    dbg = struct('active',active,'inComm',inComm,'phiRef',phiRef,'Rk',Rk,'Rdot',Rdot);
else
    dbg = [];
end

end

function wOut = phaseSpeedCoordinator(phiAct, w0, dPhiDes, k, uMax, epsPhi, wMin, wMax)
M = numel(phiAct);
wOut = w0*ones(M,1);
if M < 2, return; end
u = zeros(M,1);
for a = 1:M
    for b = 1:M
        if b==a, continue; end
        dphi = wrapToPiCustom(phiAct(a) - phiAct(b));
        ad   = abs(dphi);
        if ad < dPhiDes
            u(a) = u(a) + k * (dPhiDes - ad) * tanh(dphi/epsPhi);
        end
    end
end
u = u - mean(u);
u = clamp(u, -uMax, uMax);
wOut = clamp(w0 + u, wMin, wMax);
end

function s = refSpeedScale(dClear, clearHard, clearStart, p)
if ~isfinite(dClear), s = 1.0; return; end
if dClear >= clearStart
    s = 1.0;
elseif dClear <= clearHard
    s = 0.0;
else
    s = (dClear - clearHard) / (clearStart - clearHard);
end
s = clamp(s, 0.0, 1.0).^p;
end

function dMin = minClearanceEnv(x, y, envXY, envR, rBot)
if isempty(envXY) || isempty(envR), dMin = inf; return; end
envR = envR(:).';
dMin = inf;
for j = 1:size(envXY,2)
    d = hypot(x - envXY(1,j), y - envXY(2,j)) - (rBot + envR(j));
    if d < dMin, dMin = d; end
end
end

function A = force2xN(A)
if size(A,1)==2, return; end
if size(A,2)==2, A = A.'; return; end
error('posAll must be 2xN or Nx2.');
end

function need(S,f)
if ~isfield(S,f), error('Missing field: %s', f); end
end