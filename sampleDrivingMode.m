function [mode, laneKeeping] = sampleDrivingMode(x,Z)
% Function samples the driving mode to generate a trajectory for in each
% planning iteration. Implements equation (49) from report.

% Inputs:
%  x: Current ego vehicle state vector
%  Z: Structure containing all road information with relevant fields:
%   Z.sameLaneObsDist:  distance to obstacle occupying same lane as ego
%   Z.currLaneLabel:    string that identifies current ego vehicle lane

% Outputs:
%  mode:                sampled driving mode string
%  lanekeeping:         probability of sampling lane-keeping

    % Extract parameters from inputs
    dist = Z.sameLaneObsDist;
    EVlane = Z.currLaneLabel;
    Vel = x(4);
    
    % Set parameters for LK sampling in equation (50)
    LKBaseProb = 0.9;
    LKMinProb = 0.1;
    
    % Equation (50)
    if dist < Vel
        laneKeeping = (-(LKBaseProb-LKMinProb)/Vel^2)*(dist-Vel)^2 + LKBaseProb;
    else
        laneKeeping = LKBaseProb;
    end
    
    % Pare down feasible set of modes based on current lane
    switch(EVlane)
        case("left")
            CLL = 0;
            CLR = 1-laneKeeping;
        case("right")
            CLR = 0;
            CLL = 1-laneKeeping;
    end
    
    % Sample from uniform distribution
    sample = rand(1);
    if sample < laneKeeping
        mode = "LK";
    elseif sample >= laneKeeping && sample < laneKeeping + CLL
        mode = "CLL";
    elseif sample >= laneKeeping + CLL && sample < laneKeeping + CLL + CLR
        mode = "CLR";
    end
end