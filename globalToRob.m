% Function transforms from global frame to robot frame
% Inputs:
% robotPose, 3x1 [x;y;theta]
% point, 2x1 [x;y]

function landmark_sensor_frame = globalToRob(currentPose, landmark)
    % Transform landmark position from global frame to robot frame
    theta = currentPose(3);
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    t = currentPose(1:2);
    landmark_sensor_frame = R' * (landmark - t);
    % Debug statement to check the transformation
    disp(['Global landmark: [', num2str(landmark'), '] to Sensor frame: [', num2str(landmark_sensor_frame'), ']']);
end