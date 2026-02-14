function y = clamp(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
end
