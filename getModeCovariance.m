function [Z] = getModeCovariance(Z)
% Function to assign unique covariances according to each driving mode
% Implements equations (52) and (53)

% Input:
%  Z: Structure containing all road information with relevant fields:
%   Z.Q: Virtual system process noise covariance matrix
%   Z.R: Measurement noise covariance matrix

% Output:
%  Z: Structure containing all road information with relevant fields:
%   Z.Q: Virtual system process noise covariance matrix
%   Z.R: Measurement noise covariance matrix

    % Initialize process noise covariance Q
    Q = 1e-4*eye(7);
    
    % Adjust terms for steering angle and heading angle
    Q(3,3) = 1e-7;
    Q(5,5) = 1e-7;
    
    % Set multiplier G
    mult = 10;
    
    % Switch which covariances are used depending on sampled driving mode
    switch(Z.mode)
        case("LK")
            Qu = [1 0;
                  0 .005];
            Z.Q = Q;
            Z.Q(6:7,6:7) = Qu;
            Z.R = diag([5, 0.025, 0.1, 0.1, 0.1, 0.0002]);
            
            Z.Q = Z.Q*mult;
            Z.R = Z.R*mult;
        case("CLL")
            Qu = [1 0;
                  0 .01];
            Z.Q = Q;
            Z.Q(6:7,6:7) = Qu;
            Z.R = diag([5, 0.1, 0.1, 0.1, 0.1, 0.0007]);
    
            Z.Q = Z.Q*mult;
            Z.R = Z.R*mult;
        case("CLR")
            Qu = [1 0;
                  0 .01];
            Z.Q = Q;
            Z.Q(6:7,6:7) = Qu;
            Z.R = diag([5, 0.1, 0.1, 0.1, 0.1, 0.0007]);
    
            Z.Q = Z.Q*mult;
            Z.R = Z.R*mult;
    end
end