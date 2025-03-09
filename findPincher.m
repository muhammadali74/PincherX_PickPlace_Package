function [x,y,z,R, jointAngles] = findPincher()

    % connect arbotix pincher
    arb = Arbotix('port', 'COM4', 'nservos', 5);

    % get the current joint angles
    jointAngles = arb.getpos();

    % find end effector pos,rot
    [x,y,z,R] = pincherFK(jointAngles);

    % Display (for debugging)
    x = x/2.54;
    y = y/2.54;
    z = z/2.54;
    R;

    % disconnet the arm after use
    arb.disconnect;

end