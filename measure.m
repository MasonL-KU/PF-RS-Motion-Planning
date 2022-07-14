function out = measure(xk,Z)
% Measurement function

% Inputs:
%  xk:          state vector - single particle's state estimate
%  Z:  structure containing road information with relevant fields:
%   Z.laneRef:  reference lane centerline coordinate

% Output:
%  out:         measurement vector to compare to reference

    % Extract parameters from inputs
    laneRef = Z.laneRef;
    
    % Set parameters for Softplus barrier functions
    % a,b for obstacle avoidance
    a = 5;
    b = -10;
    % c,d for road boundary avoidance
    c = 5;
    d = 110;
    
    % get y-position of the particle state
    ys = xk(2);
    
    % Get obstacle region edge-to-edge distances
    do = getEllipseDistance(xk,Z);
    
    % eq (47) Softplus barrier function
    go = @(x) 1/a * log(1 + exp(b*x)); 

    % eq (48) Softplus barrier function
    ge = 1/c * log(1 + exp(d*(abs(ys)-3.675))); 

    % Concatenate measurements into measurement vector [nv x 1]
    out = [xk(4,:); abs(xk(2,:) - laneRef); go(do(1)); go(do(2)); ge; xk(3,:)]';
end