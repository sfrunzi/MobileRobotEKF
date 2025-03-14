% filepath: c:\Users\saf359\Documents\MATLAB\MobileRobotEKF\Sensor_def.m
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
        function scanData = emptyScan(sensor)
            num_readings = round(sensor.fov / sensor.ang_res);
            scanData = zeros(num_readings, 2);
            for i = 1:num_readings
                angle = -sensor.fov/2 + sensor.ang_res * (i - 1);
                scanData(i, 1) = angle;
                scanData(i, 2) = sensor.range_max;
            end
        end

        % returns list (scanData) with angle and range measured (dist to
        % object or range_max if no object found)
        function scanData = scan(sensor, currentPose, landmarks)
            num_readings = round(sensor.fov / sensor.ang_res);
            scanData = zeros(num_readings, 2);
            for i = 1:num_readings
                angle = -sensor.fov/2 + sensor.ang_res * (i - 1);
                scanData(i, 1) = angle;
                min_range = sensor.range_max;
                for j = 1:size(landmarks, 1)
                    % Transform landmark position to sensor frame
                    landmark = landmarks(j, :)';
                    landmark_sensor_frame = globalToRob(currentPose, landmark);
                    x = landmark_sensor_frame(1);
                    y = landmark_sensor_frame(2);
                    % Debug statement to check the transformed landmark position
                    disp(['Landmark ', num2str(j), ' in sensor frame: [', num2str(x), ', ', num2str(y), ']']);
                    % Check if landmark is within the current angle's beam
                    angle_to_landmark = atan2(y, x);
                    angle_diff = abs(angle_to_landmark - angle);
                    % Normalize angle difference to be within [-pi, pi]
                    angle_diff = mod(angle_diff + pi, 2*pi) - pi;
                    if angle_diff <= sensor.ang_res / 2
                        range = sqrt(x^2 + y^2);
                        % Debug statement to check the range and angle
                        disp(['Angle to Landmark: ', num2str(angle_to_landmark), ', Range: ', num2str(range), ', Angle Diff: ', num2str(angle_diff)]);
                        % Check if the landmark is within the sensor's range
                        if range >= sensor.range_min && range <= sensor.range_max
                            if range < min_range
                                min_range = range;
                            end
                        end
                    end
                end
                scanData(i, 2) = min_range;
                % Debug statement to check the scan data
                disp(['Angle: ', num2str(angle), ', Range: ', num2str(min_range)]);
            end
        end
    end
end

