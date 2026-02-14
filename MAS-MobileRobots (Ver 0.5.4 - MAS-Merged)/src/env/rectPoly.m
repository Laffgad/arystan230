function p = rectPoly(x, y, w, h)
    p = polyshape([x x+w x+w x], [y y y+h y+h]);
end
