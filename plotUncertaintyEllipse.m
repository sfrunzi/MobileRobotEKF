function h = plotUncertaintyEllipse(mean, covar)
    [eigvec, eigval] = eig(covar(1:2, 1:2));
    theta = linspace(0, 2*pi, 100);
    ellipse = [cos(theta); sin(theta)];
    ellipse = eigvec * sqrt(eigval) * ellipse;
    h = plot(mean(1) + ellipse(1,:), mean(2) + ellipse(2,:), 'r--');
end