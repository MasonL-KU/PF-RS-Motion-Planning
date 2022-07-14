function modeSet = getModeSet(currentLane)
% Function to pare down feasible set of modes depending on which lane the
% ego vehicle is in.
    switch(currentLane)
        case(-1.8375)
            modeSet = ["LK","CLL"];
        case(1.8375)
            modeSet = ["LK","CLR"];
    end
end