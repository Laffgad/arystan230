function [envXY, envR] = packCirclesForRepulsion(circles)
if isempty(circles)
    envXY = zeros(2,0);
    envR  = zeros(1,0);
    return;
end
envXY = reshape([circles.c], 2, []);
envR  = [circles.r];
end