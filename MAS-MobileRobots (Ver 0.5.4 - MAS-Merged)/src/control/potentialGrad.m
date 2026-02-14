function [dphi_dx, dphi_dy] = potentialGrad(x, y, obsXY, obsR, rSelf, avoid)
dphi_dx = 0; dphi_dy = 0;

for j = 1:size(obsXY,2)
    ox = obsXY(1,j); oy = obsXY(2,j);
    rx = x - ox;     ry = y - oy;
    dist = hypot(rx, ry);
    if dist < 1e-12
        dist = 1e-12; rx = 1e-12; ry = 0;
    end

    dClear = dist - (rSelf + obsR(j));
    if dClear < avoid.clearStart
        dEff = max(dClear, avoid.epsD);
        g = (1/dEff - 1/avoid.clearStart);
        coeff = (-2*g) / (dEff*dEff);
        dphi_dx = dphi_dx + coeff * (rx/dist);
        dphi_dy = dphi_dy + coeff * (ry/dist);
    end
end
end