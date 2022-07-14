function [cumCost,array] = trajectoryCost(trajectory_states,Z)
% Function evaluates trajectory costs for driving mode decision-making
% Implements equation (51) from report

% Inputs:
%  trajectory_states: smoothing trajectory over planning horizon (nx x Tp)
%  Z: Structure containing environment information with relevant fields:
%   Z.obs:      obstacle position
%   Z.vnom:     nominal velocity
%   Z.laneRef:  target lane centerline coordinate

% Output:
%  cost: cumulative cost of the input trajectory

    % Extract parameters from inputs
    x = trajectory_states;  % Renamed for code clarity
    vnom = Z.vnom;          % Nominal velocity
    laneRef = Z.laneRef;    % Reference lane centerline coordinate
    vcov = Z.R(1);          % Velocity tracking measurement noise covariance
    
    % Weights
    alpha = .75;            % Set weight for lane tracking error term
    gamma = 1/vcov;         % Adjust weight for velocity error term
    
    % Lane tracking quadratic error term
    tracking_error = alpha*abs(x(2,:)-laneRef).^2;
    
    % Get ellipse distance to input to softplus
    do = getEllipseDistance(trajectory_states,Z);
    
    % Obstacle Term (Softplus barrier function)
    a = .01;
    b = -4;
    go = @(x) (1/a * log(1 + exp(b*(x-0.25))));

    % Velocity tracking term
    velocity_error = gamma*abs(x(4,:)-vnom);
    
    % Softplus barrier function parameters for road boundary avoidance
    c = 0.2;
    d = 15;
    h = .75;
    
    % Get all y-positions along trajectory
    ys = x(2,:);
    
    % Evaluate barrier function
    ge = 1/c * log(1 + exp(d*(abs(ys)-3.675+h)));
    
    % Cumulative cost
    cumCost = sum([tracking_error, go(do(1,:)), go(do(2,:)), velocity_error, ge],'all');
    
    % Cost at each step in trajectory (1 x Tp)
    array = sum([tracking_error, go(do(1,:)), go(do(2,:)), velocity_error, ge],2);
    
    % Troubleshooting NaN costs
    % if sum(isnan(array))~=0
    %     ;
    % end
    
    % Troubleshooting infinite costs
    % if any(array == inf)
    %     ;
    % end
end