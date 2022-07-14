function region = getObstacleEllipse(x,obs)
% Function to generate elliptical obstacle avoidance region

% Inputs:
%  x:   ego vehicle state vector [nx x 1]
%  obs: obstacle position vector [Nobs x 3]

% Output:
%  region: matrix containing ellipse parameters for each obstacle in each
%   column [4 x Nobs]

    % Get parameters from inputs
    v = x(4);                   % Longitudinal velocity
    numObs = size(obs,1);       % Number of obstacles
    region = zeros(4,numObs);   % Initialize memory for output

    % Generate ellipse parameters
    b = 1.8;
    c = 0.8*v;              % eq (38)
    a = sqrt(b^2 + c^2);    % eq (39)
    
    % For each obstacle, generate ellipse parameters adjusted to obstacle
    % positions
    for i = 1:numObs
        ox = obs(i,1);
        oy = obs(i,2);
        
        % eq (40)
        regionCenter = [ox + v/2 - c;
                        oy];
        region(:,i) = [regionCenter;a;b];
    end
end


