% filepath: c:\Users\saf359\Documents\MATLAB\MobileRobotEKF\MCL_hw2.m
% Sara Frunzi, ECE 609, Winter 2024
% Extended Kalman Filter Homework 2-1
% Monte Carlo Localization with Particle Filtering

%% Define Objects
% Define Sensor 
%        Sensor_def(max, min,  fov,  noise_range, noise_ang,  ang_res,   range_res);
sensor = Sensor_def(1.5, 0.05, pi/3, 0.05       , deg2rad(1), deg2rad(1), 0.01);

% Define Robot
%       Robot_def(width, length, pose           , dia  , rightSpeed, leftSpeed)
robot = Robot_def(0.2  , 0.2   , [0.01; 0.02; 0], 0.075, 0         , 0);

% Define World
% waypoints and landmarks = [x1, y1; x2, y2; ...]
waypoints = [0.2, 0.2; 0.2, 0.4; 0.2, 0.6; 0.2, 0.8; % path forms a rough square
             0.4, 0.8; 0.6, 0.8; 0.8, 0.8;
             0.8, 0.6; 0.8, 0.4; 0.8, 0.4; 0.8, 0.3;
             0.6, 0.2; 0.4, 0.2; 0.2, 0.2];
landmarks = [0.1, 0.25; 0.1, 0.5; 0.1, 0.9;
             0.49, 0.9; 0.4, 0.6; 0.45, 0.45; 0.4, 0.32; 0.45, 0.1;
             0.63, 0.35; 0.61, 0.56; 
             0.92, 0.74; 0.91, 0.48];
%       World_def(x_min, x_max, y_min, y_max, res , wpts     , lmks)
world = World_def(0    , 1    , 0    , 1    , 0.01, waypoints, landmarks);

%% Parameters for Particle Filter
numParticles = 100; % Number of particles
particles = repmat(robot.pose, 1, numParticles) + randn(3, numParticles) * 0.1; % Initialize particles
weights = ones(1, numParticles) / numParticles; % Initialize weights

%% Run Algorithm & Plot (optional)
plotMap = true;
simTime = 5000; % ms
nextWypt = waypoints(1,:);

% Initial Scan + Gaussian Noise
% note to self: repmat(thingToRepeat, rows, cols)
lastScan = scan(sensor, robot.pose, landmarks);
noise = repmat(sensor.noise', size(lastScan, 1), 1) .* randn(size(lastScan));
lastScan = lastScan + noise; % Add Gaussian noise to the initial scan

lastPose = robot.pose;

% Initialize arrays for plotting
trueTrajectory = zeros(simTime, 3);
particleTrajectory = zeros(simTime, 3);
uncertainties = zeros(3, 3, simTime);

if plotMap
    figure;
    hold on;
    % Plot landmarks
    if ~isempty(landmarks)
        plot(landmarks(:,1), landmarks(:,2), 'ro', 'MarkerSize', 10, 'DisplayName', 'Landmarks');
    end
    % Plot waypoints
    if ~isempty(waypoints)
        plot(waypoints(:,1), waypoints(:,2), 'bx-', 'LineWidth', 2, 'DisplayName', 'Waypoints');
    end
    % Initialize plot handles for trajectories
    trueTrajPlot = plot(NaN, NaN, 'g-', 'LineWidth', 2, 'DisplayName', 'True Trajectory');
    particleTrajPlot = plot(NaN, NaN, 'k--', 'LineWidth', 2, 'DisplayName', 'Particle Trajectory');
    % Initialize plot handle for uncertainty ellipses
    uncertaintyPlot = plot(NaN, NaN, 'r--');
    % Initialize plot handle for particles
    particlesPlot = plot(NaN, NaN, 'b.', 'DisplayName', 'Particles');
    % Initialize plot handle for observations
    observationsPlot = plot(NaN, NaN, 'm*', 'DisplayName', 'Observations');
    legend;
    xlabel('X');
    ylabel('Y');
    title('Robot Trajectory with MCL');
    drawnow; % Ensure the plot is updated
end

for i=1:length(waypoints)
    % move to next waypoint
    disp(['Moving to waypoint: [', num2str(nextWypt), ']']);
    [dS, dTheta] = robot.goToPoint(nextWypt);
    if ~(i==length(waypoints))
        nextWypt = waypoints(i+1, :);
    end

    % Prediction Step
    particles = particles + [dS * cos(particles(3,:)); dS * sin(particles(3,:)); dTheta * ones(1, numParticles)] + randn(3, numParticles) * 0.01;

    % Scan
    thisScan = scan(sensor, robot.pose, landmarks);
    noise = repmat(sensor.noise', size(thisScan, 1), 1) .* randn(size(thisScan));
    thisScan = thisScan + noise;
    
    % Measurement Update Step
    for p = 1:numParticles
        expectedScan = scan(sensor, particles(:,p), landmarks);
        weights(p) = exp(-0.5 * sum((thisScan(:) - expectedScan(:)).^2));
    end
    weights = weights / sum(weights); % Normalize weights

    % Resampling Step
    indices = randsample(1:numParticles, numParticles, true, weights);
    particles = particles(:, indices);
    weights = ones(1, numParticles) / numParticles; % Reset weights

    % Estimate pose
    estimatedPose = mean(particles, 2);

    % Store trajectories and uncertainties
    trueTrajectory(i, :) = robot.pose';
    particleTrajectory(i, :) = estimatedPose';
    uncertainties(:, :, i) = cov(particles');

    % Real-time plotting
    if plotMap
        set(trueTrajPlot, 'XData', trueTrajectory(1:i,1), 'YData', trueTrajectory(1:i,2));
        set(particleTrajPlot, 'XData', particleTrajectory(1:i,1), 'YData', particleTrajectory(1:i,2));
        set(particlesPlot, 'XData', particles(1,:), 'YData', particles(2,:));
        set(observationsPlot, 'XData', thisScan(:,1), 'YData', thisScan(:,2));
        % Update uncertainty ellipse
        delete(uncertaintyPlot);
        uncertaintyPlot = plotUncertaintyEllipse(particleTrajectory(i,1:2), uncertainties(:,:,i));
        drawnow; % Ensure the plot is updated
    end
end

function h = plotUncertaintyEllipse(mean, covar)
    [eigvec, eigval] = eig(covar(1:2, 1:2));
    theta = linspace(0, 2*pi, 100);
    ellipse = [cos(theta); sin(theta)];
    ellipse = eigvec * sqrt(eigval) * ellipse;
    h = plot(mean(1) + ellipse(1,:), mean(2) + ellipse(2,:), 'r--');
end