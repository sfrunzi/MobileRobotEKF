% Sara Frunzi, ECE 609, Winter 2024
% Extended Kalman Filter Homework 2-1
% Robot Object

classdef Robot_def < handle
    % Robot Properties
    % Assumptions: Diff Drive Robot with equally sized wheels
    properties
        wheelbase   % (m)
        length      % (m)
        pose        % [x; y; theta] (m; m; rad)
        poseEst     % [x; y; theta] (m; m; rad)
        wheelDia    % (m)
        wheelSpeeds % [left, right] rad/s
        robotSpeed  % m/s
    end

    methods
        % Constructor
        function robot = Robot_def(width, length, pose, dia, rightSpeed, leftSpeed)
            robot.wheelbase = width;
            robot.length = length;
            robot.pose = pose;
            robot.poseEst = pose;
            robot.wheelDia = dia;
            robot.wheelSpeeds = [leftSpeed, rightSpeed];

            % Calculate Robot Speed in m/s
            vLeft = robot.wheelDia/2 * robot.wheelSpeeds(1);
            vRight = robot.wheelDia/2 * robot.wheelSpeeds(2);
            robot.robotSpeed = mean([vLeft, vRight]);
        end
        
        % Drive forward
        function dSTotal = driveFwd(robot, targetPose, robotSpeed)
            theta = robot.poseEst(3);
            robot.robotSpeed = robotSpeed;

            % Assume wheel speeds are the same and wheels are equidistant
            % on robot body
            robot.wheelSpeeds(1) = robot.robotSpeed / (robot.wheelDia/2);
            robot.wheelSpeeds(2) = robot.wheelSpeeds(1);

            t_start = tic;
            dSTotal = 0;
            % Increase tolerance for position updates
            tolerance = 1e-2;
            maxIterations = 10000000; % Limit the number of iterations to prevent infinite loop
            iteration = 0;
            while ~(abs(robot.poseEst(1) - targetPose(1)) < tolerance && abs(robot.poseEst(2) - targetPose(2)) < tolerance)
                dt = toc(t_start);
                t_start = tic;
                deltaS = robot.robotSpeed * dt;
                % Update x and y positions
                robot.poseEst(1) = robot.poseEst(1) + cos(theta) * deltaS;
                robot.poseEst(2) = robot.poseEst(2) + sin(theta) * deltaS;
                dSTotal = dSTotal + deltaS;
                iteration = iteration + 1;
                if iteration > maxIterations
                    disp('driveFwd: Maximum iterations reached');
                    break;
                end
            end
            % Update the robot's actual pose
            robot.pose = robot.poseEst;
            disp(['driveFwd: Reached target pose: [', num2str(robot.pose'), ']']);
        end

        function dThetaTotal = pointTurn(robot, goalHeading, robSpinSpeed) % CW = negative, CCW = positive
            % L/2 * omega = vWheel = wheelSpeed * wheelDia/2
            robot.wheelSpeeds(1) = -robot.wheelbase/2 * robSpinSpeed / (0.5 * robot.wheelDia);
            robot.wheelSpeeds(2) = -robot.wheelSpeeds(1);
            t_start = tic;
            dThetaTotal = 0;
            % Increase tolerance for heading updates
            tolerance = 1e-1;
            maxIterations = 500000; % Limit the number of iterations to prevent infinite loop
            iteration = 0;
            while abs(robot.poseEst(3) - goalHeading) > tolerance
                dt = toc(t_start);
                t_start = tic;
                deltaTheta = robSpinSpeed * dt;
                robot.poseEst(3) = robot.poseEst(3) + deltaTheta;
                dThetaTotal = dThetaTotal + deltaTheta;
                iteration = iteration + 1;
                if iteration > maxIterations
                    disp('pointTurn: Maximum iterations reached');
                    break;
                end
            end
            % Update the robot's actual pose
            robot.pose = robot.poseEst;
            disp(['pointTurn: Reached goal heading: ', num2str(robot.pose(3))]);
        end

        function dTheta = turnToWaypoint(robot, waypoint)
            % get angle to goal
            waypoint_x = waypoint(1);
            waypoint_y = waypoint(2);
            robot_x = robot.pose(1);
            robot_y = robot.pose(2);
            currHeading = robot.pose(3);

            diff_x = waypoint_x - robot_x;
            diff_y = waypoint_y - robot_y;
            
            % Get angle to waypoint between [-pi, pi]
            angleToWaypoint = getPiToPi(atan2(diff_y, diff_x));

            % determine turn direction, turn
            if (angleToWaypoint - currHeading) > pi
                dTheta = robot.pointTurn(angleToWaypoint, -1);
            else
                dTheta = robot.pointTurn(angleToWaypoint, 1);
            end
            disp(['turnToWaypoint: Turning to waypoint: [', num2str(waypoint), '] with heading: ', num2str(angleToWaypoint)]);
        end

        function [dS, dTheta] = goToPoint(robot, point, endOrientation)
            dTheta = robot.turnToWaypoint(point);
            dS = robot.driveFwd(point, 1);

            if nargin == 3 % if endOrientation is specified...
                % determine turn direction, turn
                if (endOrientation - robot.poseEst(3)) > pi
                    dTheta = dTheta + robot.pointTurn(endOrientation, -1);
                else
                    dTheta = dTheta + robot.pointTurn(endOrientation, 1);
                end
            end
            disp(['goToPoint: Reached point: [', num2str(robot.pose'), ']']);
        end
    end
end
