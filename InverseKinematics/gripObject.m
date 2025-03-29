function success = gripObject(position, grip)
    %%%%%%%%% Grip an object my moving the jaws %%%%%%%%%%%
    % Args:
    % position : int [1.4, 3.8]. the length of jaw opening --> dimension of
    % object to be gripped
    % grip: bool (0,1) 1 if you want to grip object
    delta = 0.29;
    P = 15;
    griptheta = 0;
    if grip == 1
        griptheta = P*delta;
    end

    success = positionJaw(position, griptheta);

end

function success = positionJaw(position, griptheta)
    global arb
    if (position > 3.8) || (position < 1.4)
        success = -1;
        return;
    end
    
    delta = 0.02;
    theta = (3.8 - position) / delta
    
    arb.setpos(5,deg2rad(theta+griptheta), 50)
    success = 0;

end

