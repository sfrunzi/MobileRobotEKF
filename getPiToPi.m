function angle = getPiToPi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end