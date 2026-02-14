function plotCircle(c, r, ls, lw)
    if nargin < 3 || isempty(ls), ls = 'r-'; end
    if nargin < 4 || isempty(lw), lw = 1.0; end
    th = linspace(0, 2*pi, 200);
    x = c(1) + r*cos(th);
    y = c(2) + r*sin(th);
    plot(x, y, ls, 'LineWidth', lw);
end
