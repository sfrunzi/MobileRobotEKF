% Function transforms from robot frame to global frame
% Inputs:
% robotPose, 3x1 [x;y;theta]
% point, 2x1 [x;y]
function pointGlobal = robToGlobal(robotPose, point)
    % get angle between -pi, pi
    % robotPose(theta) is offset by pi/2 (ie- pi/2 in global frame is 0 in
    % robot frame)
    theta = getPiToPi(robotPose(3) - pi/2);
    
    % Rotation Matrix
    R = [cos(theta), -sin(theta) ; 
        sin(theta), cos(theta)];

    pointGlobal = R * point;

end

