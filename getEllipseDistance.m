function [edgeDist] = getEllipseDistance(particle,Z)
% Function calculates the edge-to-edge distance between ellipses. Each
% particle has an ellipse that bounds it, and each obstacle has an
% elongated obstacle region.

% Inputs:
%  particle: state vectors for particle(s)
%  Z: field containing relevant information
%   Z.obs: obstacle position
%   Z.obstacleRegion: 4 x Nobs matrix
%    1st 2 rows of each column are ellipse center coordinate
%    3rd row is ellipse parameter a (major radius)
%    4th row is ellipse parameter b (minor radius)

% Output:
%  edgeDist: array of scalar distances between each particle ellipse and
%   obstacle avoidance region ellipse

    % Extract parameter values from inputs
    numObs = size(Z.obs,1);
    numStates = size(particle,2);
    obsCenters = Z.obstacleRegion(1:2,:);
    us = Z.obstacleRegion(3,:);
    vs = Z.obstacleRegion(4,:);
    
    % Initialize edgeDist output memory
    edgeDist = zeros(numObs,numStates);
    
    % Particle ellipse parameters
    % Defined based on the dimensions of the rectangular vehicle body
    a = 2.944;
    b = 1.494;
    yaw = particle(3,:);
    
    % Get edge distance for each obstacle from each particle
    for i = 1:numObs
        dx = obsCenters(1,i) - particle(1,:);
        dy = obsCenters(2,i) - particle(2,:);
        
        % Global angle between ellipses
        theta = atan(dy./dx);
        
        % Center-to-center distance between ellipses
        cdist = sqrt(dx.^2 + dy.^2);
        
        % Calculate 'specific radii' of ellipse pair
        egoSpecificRadius = a*b./sqrt((b*cos(theta+yaw)).^2 + (a*sin(theta+yaw)).^2);
        obsSpecificRadius = us(i)*vs(i)./sqrt((vs(i)*cos(theta)).^2 + (us(i)*sin(theta)).^2);

        % Calculate edge-to-edge distance for ellipse pair
        edgeDist(i,:) = cdist-egoSpecificRadius-obsSpecificRadius;
    end
end
