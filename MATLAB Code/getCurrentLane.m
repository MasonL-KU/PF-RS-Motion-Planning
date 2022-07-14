function currLaneCenter = getCurrentLane(EV)
% Function to identify the current lane center coordinate of ego vehicle

% Input:
%  EV: Structure containing ego vehicle parameters and state vector

% Output:
%  currLaneCenter: lane centerline coordinate of occupied lane

    if EV.Position(2) > 0 && EV.Position(2) < 3.675
        currLaneCenter = 3.675/2;
    elseif EV.Position(2) <= 0 && EV.Position(2) > -3.675
        currLaneCenter = -3.675/2;
    end
end