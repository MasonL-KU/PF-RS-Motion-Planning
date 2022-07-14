function [xnew, noise] = vehicle_model(xbar, dt, Q)
% Function implements kinematic bicycle model equation (7) from report
% This version applies process noise to the virtual system

% Inputs:
%  xbar:    augmented state and controls [nx x N]
%  dt:      discretization time

% Outputs:
%  xnew:    resulting augmented state [nx X N]

    % Set virtual system state vector dimension
    nx = size(Q,1);
    
    % Vehicle parameters
    L = 2.8;    % wheelbase
    lr = L/2;   % center to rear axle
    
    % Kinematic body slip angle
    beta = @(s) atan(lr*tan(s)/L);
    
    % Dynamics
    xdot = @(x) [x(4)*cos(x(3)+beta(x(5)))/cos(beta(x(5)));
                 x(4)*sin(x(3)+beta(x(5)))/cos(beta(x(5)));
                 x(4)*tan(x(5))/L;
                 x(6);
                 x(7);
                 0;
                 0];
    
    % Equation (7)
    noise = mvnrnd(zeros(nx,1),Q)';
    xnew = xbar + xdot(xbar).*dt + noise;
end