function p = circlePoly(cx, cy, r, n)
    if nargin < 4
        n = 64;
    end
    th = linspace(0, 2*pi, n+1);
    th(end) = [];
    x = cx + r*cos(th);
    y = cy + r*sin(th);
    p = polyshape(x, y);
end
