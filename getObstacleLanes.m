function lane_idx = getObstacleLanes(Z)
% Function identifies which lane each obstacle is occupying

% Input: 
%  Z: Structure containing all road information with relevant fields:
%   Z.obs:  obstacle positions

% Output: 
%  lane_idx: Structure with indices of which obstacle is in which lane

    % Extract lateral position of each obstacle
    ys = Z.obs(:,2);
    
    % Identify which obstacle are in which lane with binary variables
    LL = ys > 0;
    RL = ys <= 0;
    
    % Indexes which obstacles are in which lane
    % If leftObsIdx returns 1, obstacle 1 is in left lane
    leftObsIdx = find(LL);
    rightObsIdx = find(RL);
    
    lane_idx.left = leftObsIdx;
    lane_idx.right = rightObsIdx;
end