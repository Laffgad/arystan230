function [cmd, st, dbg] = formationHighLevelStep(leaderPos, vPlan, envXY, envR, st, P)

need(P,'dt'); need(P,'robotRadius'); need(P,'leaderRadius');
need(P,'form'); need(P.form,'Rmin'); need(P.form,'Rmax'); need(P.form,'tauR');
need(P.form,'vLeadMax'); need(P.form,'safety');
need(P.form,'w0'); need(P.form,'minW');
need(P.form,'vSlow'); need(P.form.vSlow,'clearHard'); need(P.form.vSlow,'clearStart'); need(P.form.vSlow,'p');

dt = P.dt;
rR = P.robotRadius;
rL = P.leaderRadius;

if nargin < 5 || isempty(st), st = struct(); end
if ~isfield(st,'Rprev') || ~isfinite(st.Rprev), st.Rprev = P.form.Rmax; end

vPlan = clamp(vPlan, 0, P.form.vLeadMax);

lx = leaderPos(1); ly = leaderPos(2);

if isempty(envXY) || isempty(envR)
    Rdes = clamp(P.form.Rmax, P.form.Rmin, P.form.Rmax);
    Rcmd = st.Rprev + (dt/max(P.form.tauR,1e-6))*(Rdes - st.Rprev);
    Rcmd = clamp(Rcmd, P.form.Rmin, P.form.Rmax);
    Rdot = (Rcmd - st.Rprev)/dt;
    st.Rprev = Rcmd;

    cmd = struct('R',Rcmd,'Rdot',Rdot,'w0',max(P.form.minW,P.form.w0), ...
        'vLeader',vPlan,'dObsMin',inf,'dClearLeader',inf);
    dbg = cmd;
    return;
end

dCenter = hypot(lx - envXY(1,:), ly - envXY(2,:));
dToObs  = dCenter - envR(:).';
dObsMin = min(dToObs);

dClearLeader = dObsMin - rL;

Rallow = dObsMin     - (rL + rR) - P.form.safety;
Rdes   = clamp(Rallow, P.form.Rmin, P.form.Rmax);

Rcmd = st.Rprev + (dt/max(P.form.tauR,1e-6))*(Rdes - st.Rprev);
Rcmd = clamp(Rcmd, P.form.Rmin, P.form.Rmax);

Rdot = (Rcmd - st.Rprev)/dt;
st.Rprev = Rcmd;

sV = speedScale(dClearLeader, P.form.vSlow.clearHard, P.form.vSlow.clearStart, P.form.vSlow.p);
vLeader = clamp(vPlan * sV, 0.2, P.form.vLeadMax);

w0cmd = max(P.form.minW, P.form.w0 * sV);

cmd = struct('R',Rcmd,'Rdot',Rdot,'w0',w0cmd,'vLeader',vLeader, ...
    'dObsMin',dObsMin,'dClearLeader',dClearLeader);
dbg = cmd;

end

function s = speedScale(dClear, hard, start, p)
if ~isfinite(dClear), s = 1; return; end
if dClear >= start
    s = 1;
elseif dClear <= hard
    s = 0;
else
    s = (dClear - hard) / (start - hard);
end
s = clamp(s, 0, 1).^p;
end

function need(S,f)
if ~isfield(S,f), error('Missing field: %s', f); end
end