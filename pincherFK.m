    
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
