% Sara Frunzi, ECE 609, Winter 2024
% Extended Kalman Filter Homework 2-1
% Main File

%% Define Objects
% Define Sensor 
Sensor_def sensor;
%        Sensor_def(max, min , fov , noise_range, noise_ang , ang_res);
sensor = Sensor_def(1.5, 0.05, pi/3, 0.05       , deg2rad(1), deg2rad(1));

% Define Robot
Robot_def robot;
%       Robot_def(width, length, pose           , dia  , rightSpeed, leftSpeed)
robot = Robot_def(0.2  , 0.2   , [0.01; 0.02; 0], 0.075, 0         , 0);

% Define World
World_def world;
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
plot = false;
simTime = 5000; % ms
nextWypt = waypoints(1,1);
prevPredict = eye(3);

% Initial Scan + Gaussian Noise
% note to self: repmat(thingToRepeat, rows, cols)
lastScan = scan(sensor, robot.pose, landmarks);
noise = repmat(sensor.noise, 1, length(lastScan)) .* randn(2,length(lastScan));
lastScan = lastScan + noise;

lastPose = robot.pose;

for i=1:length(waypoints)
    % move to next waypoint
    [dS, dTheta] = robot.goToPoint(nextWypt);
    if ~(t==length(waypoints))
        nextWypt = waypoints(i+1, i+1);
    end

    % Scan
    thisScan = scan(sensor, robot.pose, landmarks);
    noise = repmat(sensor.noise, 1, length(thisScan)) .* randn(2,length(thisScan));
    thisScan = thisScan + noise;
    
    thisPoseEst = robot.poseEst;

    % Prediction Step
    % Note to self: if this doesn't work, try with dTheta instead of
    % poseEst
    F = [1, 0, -dS*sin(robot.poseEst(3));
         0, 1,  dS*cos(robot.poseEst(3));
         0, 0, 1];

    B = [cos(robot.poseEst(3)), -dS*sin(robot.poseEst(3));
         sin(robot.poseEst(3)),  dS*cos(robot.poseEst(3));
         0                    , 1];


    Prediction = F*prevPredict*F';
    prevPredict = Prediction;



    
    lastScan = thisScan;
    lastPose = thisPose;

end