function [xnew] = vehicle_model_det(x, u, dt)
% Function implements kinematic bicycle model equation (2) from report
% This version does not apply process noise and is only used to control the
% true system

% Inputs:
%  x:       state vector - no controls [5 x 1]
%  u:       controls extracted from filtering/smoothing
%  dt:      discretization time

% Outputs:
%  xnew:    resulting state

    % Vehicle parameters
    L = 2.8;    % wheelbase
    lr = L/2;   % center to rear axle
    
    % Kinematic body slip angle
    beta = @(s) atan(lr*tan(s)/L);
    
    % Dynamics
    xdot = @(x,u) [x(4)*cos(x(3)+beta(x(5)))/cos(beta(x(5)));
                   x(4)*sin(x(3)+beta(x(5)))/cos(beta(x(5)));
                   x(4)*tan(x(5))/L;
                   u(1);
                   u(2);
                   0;
                   0];
    
    % Return new state
    xnew = x + xdot(x,u).*dt;
end