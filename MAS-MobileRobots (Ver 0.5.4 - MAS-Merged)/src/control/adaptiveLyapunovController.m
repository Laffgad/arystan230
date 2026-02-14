function [v,w,e,kHat,dbg] = adaptiveLyapunovController(x, xr, yr, thr, vr, wr, kHat, P, rep)

X  = x(1); Y  = x(2); th = x(3);

dx  = xr - X;
dy  = yr - Y;

ex  =  cos(th)*dx + sin(th)*dy;
ey  = -sin(th)*dx + cos(th)*dy;
eth = wrapToPiCustom(thr - th);

e = [ex; ey; eth];

if abs(eth) < 1e-6
    s = 1.0;
else
    s = sin(eth)/eth;
end

vTrk = vr*cos(eth) + kHat(1)*ex;
wTrk = wr + vr*s*(kHat(2)*ey) + kHat(3)*eth;

vTrk = clamp(vTrk, -P.vMax, P.vMax);
wTrk = clamp(wTrk, -P.wMax, P.wMax);

vRep = 0.0;
wRep = 0.0;

if nargin >= 9 && ~isempty(rep)
    if isstruct(rep) && isfield(rep,'grad')
        gradW = rep.grad(:);
    elseif isstruct(rep) && isfield(rep,'dphi_dx') && isfield(rep,'dphi_dy')
        gradW = [rep.dphi_dx; rep.dphi_dy];
    else
        gradW = rep(:);
    end

    if numel(gradW) ~= 2
        error('rep must provide a 2x1 world gradient: [dphi_dx; dphi_dy].');
    end

    if isfield(P,'avoid') && isfield(P.avoid,'gradMax')
        g = norm(gradW);
        if g > P.avoid.gradMax
            gradW = gradW * (P.avoid.gradMax / g);
        end
    end

    ko = P.avoid.ko;
    vrepW = -ko * gradW;

    c = cos(th); sTh = sin(th);
    vRep =  vrepW(1)*c + vrepW(2)*sTh;
    yRep = -vrepW(1)*sTh + vrepW(2)*c;

    wRep = yRep;
end

v = clamp(vTrk + vRep, -P.vMax, P.vMax);
w = clamp(wTrk + wRep, -P.wMax, P.wMax);

kDot = [-P.adapt.gamma(1)*ex  *(vr - vTrk);
        -P.adapt.gamma(2)*ey  *(wr - wTrk);
        -P.adapt.gamma(3)*eth *(wr - wTrk)];

kHat = kHat + P.dt*kDot;
kHat = max(kHat, P.adapt.kMin);
kHat = min(kHat, P.adapt.kMax);

if nargout >= 5
    dbg = struct('vTrk',vTrk,'wTrk',wTrk,'vRep',vRep,'wRep',wRep);
end
end