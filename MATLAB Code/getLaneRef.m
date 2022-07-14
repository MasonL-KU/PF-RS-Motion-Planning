function reference = getLaneRef(modeString,currLane)
% Function to identify goal lane coordinate

% Inputs:
%  modeString:  string for selected driving mode
%  currLane:    ego vehicle current lane centerline

% Output:
%  reference:   goal lane coordinate

    switch(modeString)
        case("LK")
            reference = currLane;
        case("CLL")
            % Change lane left adds lane width to current lane
            reference = currLane + 3.675;
        case("CLR")
            % Change lane right subtracts lane width from current lane
            reference = currLane - 3.675;
    end
end