function V = robotRectVerts(x, y, th, L, W)
    hx = L/2;
    hy = W/2;
    
    Vb = [ hx,  hy;
           hx, -hy;
          -hx, -hy;
          -hx,  hy];
    
    c = cos(th); s = sin(th);
    Rot = [c -s; s c];
    
    V = (Rot * Vb.').';
    V(:,1) = V(:,1) + x;
    V(:,2) = V(:,2) + y;
end