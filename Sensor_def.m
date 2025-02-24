% Sara Frunzi, ECE 609, Winter 2024
% Extended Kalman Filter Homework 2-1
% Sensor Object

classdef Sensor_def < handle
    properties
        range_max       % Range of laser sensor (m)
        range_min       % Min distance of sensor reading (m)
        fov             % Angle over which sensor operates (field of view)
        noise           % Measurement noise [m; rad]
        ang_res         % Angular resolution (rad)
        range_res       % Range resolution (m)
    end
    methods
        % Constructor-- define sensor object
        function sensor = Sensor_def(max, min, fov, noise_range, noise_ang, ang_res, range_res)
            sensor.range_max = max;
            sensor.range_min = min;
            sensor.fov = fov;
            sensor.noise = [noise_range; noise_ang];
            sensor.ang_res = ang_res;
            sensor.range_res = range_res;
        end

        % Empty Scan -- Entire Field of View returns range max
        function emptyScan(sensor)
            num_readings = -sensor.fov/2 : sensor.ang_res : sensor.fov/2;
            scan = zeros(num_readings, 2);
            for i = 1:num_readings
                angle = -sensor.fov/2 + sensor.ang_res*i;
                scan(i, 1) = angle;
                scan(i, 2) = sensor.range_max;
            end
        end

        % returns list (scanData) with angle and range measured (dist to
        % object or range_max if no object found)
        function scanData = scan(sensor, currentPose, landmarks)
            num_readings = -sensor.fov/2 : sensor.ang_res : sensor.fov/2;
            scanData = zeros(num_readings, 2);
            for i = 1:num_readings
                angle = -sensor.fov/2 + sensor.ang_res*i;
                scanData(i, 1) = angle;
                for j=1:sensor.range_res:sensor.range_max
                    % range from sensor POV
                    xDist = j*cos(angle); 
                    yDist = j*sin(angle);

                    % get sensor frame --> global frame
                    pointGlobal = robToGlobal(currentPose, [xDist; yDist]);
                    xLoc = pointGlobal(1);
                    yLoc = pointGlobal(2);

                    % if there is a landmark at this location or sensor is
                    % at max range, return range value
                    if (ismember(landmarks(:, 1), xLoc) && ismember(landmarks(:, 2), yLoc))
                        range = j;
                        break;
                    elseif ~(ismember(landmarks(:, 1), xLoc) && ismember(landmarks(:, 2), yLoc)) && j==sensor.range_max
                        range = j;
                    end
                end
                scanData(i, 2) = range;
            end
        end

    end
end