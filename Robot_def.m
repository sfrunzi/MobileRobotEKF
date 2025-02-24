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

            % Add out of range thetas
            % while(theta > 2*pi)
            %     theta = 
            % end
            
            if (theta == 0 || (theta <= pi/2 && theta >= 0) || (theta >= 3*pi/2 && theta <= 2*pi) || ...
                    (theta >= -pi/2 && theta <= 0) || (theta <= -3*pi/2 && theta > -2*pi))
                facingNS = 'N';
            elseif (theta == pi || (theta > pi/2 && theta < 3*pi/2) || (theta < -pi/2 && theta > -3*pi/2))
                facingNS = 'S';
            else
                facingNS = 'error';
            end

            if ((theta >= 0 && theta <= pi) || (theta <= -pi && theta <= -2*pi))
                facingEW = 'E';
            elseif ((theta > pi && theta <= 2*pi) || (theta < 0 && theta > -pi))
                facingEW = 'W';
            else
                facingEW = 'error';
            end
            
            t_start = tic;
            dt = toc;
            dSTotal = 0;
            % Add Range, not ==
            while ~(robot.poseEst(1) == targetPose(1)) && ~(robot.poseEst(2) == targetPose(2))
                deltaS = robot.robotSpeed*dt;
                % Update x position
                if (facingEW == 'E')
                    robot.poseEst(1) = robot.poseEst(1) + abs(cos(theta))*deltaS;
                elseif (facingEW == 'W')
                    robot.poseEst(1) = robot.poseEst(1) - abs(cos(theta))*deltaS;
                else
                    robot.poseEst(1) = robot.poseEst(1);
                end
                
                % Update y position
                if (facingNS == 'N')
                    robot.poseEst(2) = robot.poseEst(2) + abs(sin(theta))*deltaS;
                elseif (facingNS == 'S')
                    robot.poseEst(2) = robot.poseEst(2) - abs(sin(theta))*deltaS;
                else
                    robot.poseEst(2) = robot.poseEst(2);
                end
                dSTotal = dSTotal + deltaS;
                dt = toc;
            end
        end

        function dThetaTotal = pointTurn(robot, goalHeading, robSpinSpeed) % CW = negative, CCW = positive
            % L/2 * omega = vWheel = wheelSpeed * wheelDia/2
            robot.wheelSpeeds(1) = - robot.wheelbase/2 * robSpinSpeed/(0.5*robot.wheelDia);
            robot.wheelSpeeds(2) = - robot.wheelSpeeds(1);
            now = tic;
            dt = toc;
            dThetaTotal = 0;
            while ~(robot.poseEst(3) == goalHeading)
                deltaTheta = robSpinSpeed*dt;
                robot.poseEst(3) = robot.poseEst(3) + deltaTheta;
                dThetaTotal = dThetaTotal + deltaTheta;
                dt = toc;
            end
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
            angleToWaypoint = getPiToPi(atan(diff_y/diff_x));

            % determine turn direction, turn
            if (angleToWaypoint - currHeading) > pi
                dTheta = pointTurn(robot, angleToWaypoint, -1)
            else
                dTheta = pointTurn(robot, angleToWaypoint, 1)
            end
        end

        function [dS, dTheta] = goToPoint(robot, point, endOrientation)
            dTheta = turnToWaypoint(robot, point);
            dS = driveFwd(robot, point, 1);

            if nargin < 2 % if endOrientation is specified...
                % determine turn direction, turn
                if (endOrientation - robot.poseEst(3)) > pi
                    dTheta = dTheta + pointTurn(robot, endOrientation, -1);
                else
                    dTheta = dTheta + pointTurn(robot, endOrientation, 1);
                end
            end
        end
    end
end