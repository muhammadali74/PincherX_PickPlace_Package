function q = findOptimalSolution(x,y,z,phi)
    %%%%%%%%%%%% Find optimal IK solution fro the given pose %%%%%%%%%%%%%
    % Args: 
    % (x,y,z) : desired position of end effector 
    % phi : angle of the end effector wrt horizontal plane

    % make sure to initialize arb variable before using this function.
    global arb
    % [cx,cy,cz, cR, q0] = findPincher();
    % Find current joint angles
    q0 = arb.getpos();
    q0 = q0(1:4);
    Q = findJointAngles(x,y,z,phi);

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
    params = [b1 b1/2 b1/4 b1/8].'; % the farther the joint from endeffector the higher the penalty

    d_t = d_t.*params;

    errors = sqrt(sum(d_t, 1));

    [minval, col] = min(errors);
    q = valid_Q(:, col);

end

function A = normalize(angle)
    A = mod(angle + pi, 2*pi) - pi;

end

function Q = findJointAngles(x,y,z,phi)
    %%%%%%% Find all possible IK solutions for a pose %%%%%%%%%%%%%%%%
    L12 = 14;
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


%% HELPER FUNCS %%%%%%%%%

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

