

        % function initArb()
        %     utils.arb = Arbotix('port', 'COM4', 'nservos', 5)
        %     pause(10);
        % 
        % end

     function [x,y,z,R, jointAngles] = findPincher()

            global arb
            jointAngles = arb.getpos();
            [x,y,z,R] = pincherFK(jointAngles);

        end

    function success = positionJaw(position, griptheta)
        if (position > 3.8) || (position < 1.4)
            success = -1;
            return;
        end
        
        delta = 0.02;
        theta = (3.8 - position) / delta;
        global arb
        arb.setpos(5,deg2rad(theta+griptheta), 50)
        success = 0;
    
    end

    function success = gripObject(position)
        delta = 0.29;
        P = 15;
        griptheta = P*delta;
        success = positionJaw(position, griptheta,arb);
    
    end

    function pose = preGraspPose(pose_obj)
        if sqrt(pose_obj(2)^2 + pose_obj(1)^2) < 17.78 
            pose = [pose_obj(1); pose_obj(2); pose_obj(3) + 5; -pi/2];
        else
            pose = [pose_obj(1); pose_obj(2); pose_obj(3) + 5; 0];
        end
    end



    function q = findOptimalSolution(x,y,z,phi)
    % global arb
    [cx,cy,cz, cR, q0] = findPincher();
    q0 = q0(1:4);
    Q = findJointAngles(x,y,z,phi)

    valid_Q = [];
    % Loop through each column of the matrix
    for col = 1:size(Q, 2)

        if all(Q(:, col) >= deg2rad(-150) & Q(:, col) <= deg2rad(150))
            valid_Q = [valid_Q, Q(:, col)];

        end
    end

    rad2deg(valid_Q)

    d_t = (valid_Q - q0.').^2;
    b1 = 1;

    params = [b1 b1/2 b1/4 b1/8].';

    d_t = d_t.*params;

    errors = sum(d_t, 1);

    [minval, col] = min(errors);
    q = valid_Q(:, col);

end

function A = normalize(angle)
    A = mod(angle + pi, 2*pi) - pi;

end

function Q = findJointAngles(x,y,z,phi)

    L12 = 12;
    L3 = 10.6;
    L4 = 10.6;
    L5 = 7.4;

    r = sqrt(x^2 + y^2);
    s = z - L12;
    u = round(r - L5*cos(phi), 10);
    v = round(s - L5*sin(phi), 10);

    theta_1 = [atan2(y,x); pi + atan2(y,x)] ;
    t1 = rad2deg(theta_1);

    D = (u^2 + v^2 - L3^2 - L4^2)/(2*L3*L4);


    theta_3 = [atan2(sqrt(1-D^2), D)  atan2(-sqrt(1-D^2), D)];
    t3 = rad2deg(theta_3);

    theta_2 = atan2(v,u) - atan2(L3*sin(theta_3), L3 + L4*cos(theta_3));


    theta_4 = phi - theta_3 - theta_2;
    t4 = rad2deg(theta_4);
    


    t2 = rad2deg(theta_2);
    Q1 = normalize([theta_1(1) theta_1(1); theta_2; theta_3; theta_4] + [-pi/2;-pi/2;0;0]);

    Q2 = normalize([theta_1(2) theta_1(2); pi-theta_2; -theta_3; pi-phi - (pi-theta_2) - (-theta_3)] + [-pi/2;-pi/2;0;0]);



    Q = [Q1 Q2];

end


function errorCode = setPosition(jointAngles, speed)
    % Initialize errorCode
    errorCode = 0;
    % speed = 30;
    
   % Check if joint angles are within range
    if length(jointAngles) > 5 || length(jointAngles) < 4
        errorCode = 400; % Error for too many or too less joint angles
        return;
    end
    
    for i = 1:length(jointAngles)
        if jointAngles(i) < -2.618 || jointAngles(i) > 2.618
            errorCode = i; % Assign error code based on joint index
            return;
        end
    end
    
    % Check speed conditions
    if speed > 500
        errorCode = 100; % Speed error code
        return;
    elseif speed > 100
        warning('Speed is greater than 100, this may cause instability.');
    end
    
    % Connect Arbotix Pincher
    global arb
    % arb = Arbotix('port', 'COM4', 'nservos', 5);
    
    % Convert joint angles to radians
    % jointAngles = deg2rad(jointAngles);

    % Set joint positions
    arb.setpos(1, jointAngles(1), speed);
    arb.setpos(2, jointAngles(2), speed);
    arb.setpos(3, jointAngles(3), speed);
    arb.setpos(4, jointAngles(4), speed);


    % If jointAngles size is 5, set position for the fifth joint
    if length(jointAngles) == 5
        arb.setpos(5, jointAngles(5), speed);
    end
    
    % Disconnect the arm after use
    % utils.arb.disconnect;
end

 function [x,y,z,R] = pincherFK(jointAngles)
    
    % measured arm link lengths
    L12 = 14;
    L3 = 10.6;
    L4 = 10.6;
    L5 = 7.4;
    

    theta_1 = jointAngles(1);
    theta_2 = jointAngles(2);
    theta_3 = jointAngles(3);
    theta_4 = jointAngles(4);


    %%%%%%%%%%%% Calculation of the end effector frame %%%%%%%%%%%%%%%%%%%
    % rotation axis for joint 2 to 4
    w4_2 = [1 ;0 ;0];
    
    % rotation axis for join t
    w1 = [0; 0; 1];
    
    %L1L2 represents a single variable encompassing the length from base to arm
    %1
    q1 = [0; 0; 0];
    q2 = [0 ;0 ;L12];
    q3 = [0 ;0 ;L12 + L3];
    q4 = [0 ;0; L12 + L3 + L4];
    
    v4 = cross(-w4_2 ,q4);
    v3 = cross(-w4_2 ,q3);
    v2 = cross(-w4_2 ,q2);
    v1 = cross(-w1 ,q1);
    
    S1 = [w1 ; v1];
    S2 = [w4_2; v2];
    S3 = [w4_2; v3];
    S4 = [w4_2; v4];
    
    M = [1 0 0 0; 0 1 0 0; 0 0 1 L12 + L3 + L4 + L5;  0 0 0 1];
    
    T01 = expen(S1, theta_1);
    T12 = expen(S2, theta_2);
    T23 = expen(S3, theta_3);
    T34 = expen(S4, theta_4);
    
    T04 = T01 * T12 * T23 * T34;
    
    
    R = T04 * M;

    % extracting position and rotation from the homogenous transform
    
    pos = R(:,4);
    x = pos(1);
    y = pos(2);
    z = pos(3);
    R = R(1:3, 1:3);
    
    
    end
    
    function vs = vec2skew(v)
        vs = [0 -v(3) v(2);
              v(3) 0 -v(1);
             -v(2) v(1) 0];
    end
    
    function T = expen(S, theta)
        w_skew = vec2skew(S(1:3));
        exp_w = eye(3) + w_skew*sin(theta) + w_skew^2*(1-cos(theta));
        G = eye(3)*theta + (1-cos(theta))*w_skew+(theta-sin(theta))*w_skew^2;
        T = [exp_w, G*S(4:6);
             zeros(1,3), 1];
    end



