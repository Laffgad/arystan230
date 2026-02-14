function a = wrapToPiCustom(a)
    a = mod(a + pi, 2*pi) - pi;
end
