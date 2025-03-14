% Sara Frunzi, ECE 609, Winter 2024
% Extended Kalman Filter Homework 2-1
% Main File

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
uncertainty = eye(3); % Initial covariance matrix

% Initialize arrays for plotting
trueTrajectory = zeros(simTime, 3);
predictedTrajectory = zeros(simTime, 3);
correctedTrajectory = zeros(simTime, 3);
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
    predictedTrajPlot = plot(NaN, NaN, 'k--', 'LineWidth', 2, 'DisplayName', 'Predicted Trajectory');
    correctedTrajPlot = plot(NaN, NaN, 'b-', 'LineWidth', 2, 'DisplayName', 'Corrected Trajectory');
    % Initialize plot handle for uncertainty ellipses
    uncertaintyPlot = plot(NaN, NaN, 'r--');
    legend;
    xlabel('X');
    ylabel('Y');
    title('Robot Trajectory with EKF');
    drawnow; % Ensure the plot is updated
end

for i=1:length(waypoints)
    % move to next waypoint
    disp(['Moving to waypoint: [', num2str(nextWypt), ']']);
    [dS, dTheta] = robot.goToPoint(nextWypt);
    if ~(i==length(waypoints))
        nextWypt = waypoints(i+1, :);
    end

    % Scan
    thisScan = scan(sensor, robot.pose, landmarks);
    noise = repmat(sensor.noise', size(thisScan, 1), 1) .* randn(size(thisScan));
    thisScan = thisScan + noise;
    
    thisPoseEst = robot.poseEst;

    % Prediction Step
    F = [1, 0, -dS*sin(robot.poseEst(3));
         0, 1,  dS*cos(robot.poseEst(3));
         0, 0, 1]; % Jacobian of state transition function

    B = [cos(robot.poseEst(3)), -dS*sin(robot.poseEst(3));
         sin(robot.poseEst(3)),  dS*cos(robot.poseEst(3));
         0                    , 1]; % Jacobian of control input function

    noiseCovar = eye(3) * 0.01; % Process noise covariance
    uncertainty = F * uncertainty * F' + noiseCovar;

    % Measurement Update Step
    measMatrix = eye(3); % Measurement matrix (assuming direct observation)
    rangeNoise = sensor.noise(1); % Range noise
    bearingNoise = sensor.noise(2); % Bearing noise
    % Construct the measurement noise covariance matrix
    measNoiseCovar = diag([rangeNoise^2, bearingNoise^2, 0.01^2]); % Adjust the third element as needed
    K = uncertainty * measMatrix' / (measMatrix * uncertainty * measMatrix' + measNoiseCovar); % Kalman gain

    meas = thisScan(:); % Measurement vector
    measExpected = scan(sensor, thisPoseEst, landmarks);
    measExpected = measExpected(:); % Expected measurement vector
    y = meas - measExpected; % Measurement difference

    % Ensure y has the correct dimensions
    if length(y) ~= size(K, 2)
        y = y(1:size(K, 2));
    end

    thisPoseEst = thisPoseEst + K * y; % Update state estimate
    uncertainty = (eye(3) - K * measMatrix) * uncertainty; % Update covariance matrix

    % Store trajectories and uncertainties
    trueTrajectory(i, :) = robot.pose';
    predictedTrajectory(i, :) = thisPoseEst';
    correctedTrajectory(i, :) = robot.poseEst';
    uncertainties(:, :, i) = uncertainty;

    lastScan = thisScan;

    % Real-time plotting
    if plotMap
        set(trueTrajPlot, 'XData', trueTrajectory(1:i,1), 'YData', trueTrajectory(1:i,2));
        set(predictedTrajPlot, 'XData', predictedTrajectory(1:i,1), 'YData', predictedTrajectory(1:i,2));
        set(correctedTrajPlot, 'XData', correctedTrajectory(1:i,1), 'YData', correctedTrajectory(1:i,2));
        % Update uncertainty ellipse
        delete(uncertaintyPlot);
        uncertaintyPlot = plotUncertaintyEllipse(correctedTrajectory(i,1:2), uncertainties(:,:,i));
        drawnow; % Ensure the plot is updated
    end
end

