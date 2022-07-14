function [xh, pf] = particlefilter(r,pf,Z)
% Function implements bootstrap particle filter (Algorithm 1)

% Inputs:
%  r: reference vector
%  pf: particle filter structure with relevant fields:
%   pf.particles:   particles (nx x N x Tp)
%   pf.N:           particle count
%   pf.wf:          filtering weights (N x Tp)
%   pf.Tp:          planning horizon length
%  Z: Structure containing all road information with relevant fields:
%   Z.Q:            process noise covariance (nx x nx)

% Outputs:
%  xh: filtering state trajectory estimate
%  pf: particle filter structure with updated filtering weights
%   pf.wf:          updated filtering weights
%   pf.particles:   particles propagated through system dynamics

    % Extract parameters from inputs
    k = pf.k;
    dt = pf.dt;
    N = pf.N;
    Q = Z.Q;
    Tp = pf.Tp;

    % Sample initial particle set
    if k == 2
        pf.particles(:,:,1) = pf.gen_x0(pf.x0,pf.N,Q)';
    end

    for i = 1:N
        % Propagate Dynamics
        pf.particles(:,i,k) = vehicle_model(pf.particles(:,i,k-1),dt,Q);
        
        % Weight update
        pf.wf(i,k) = pf.wf(i,k-1) * pf.p_y_given_x(r,pf.particles(:,i,k),Z);

        % Assign zero weight to NaN filtering weights
        if isnan(pf.wf(i,k))
            pf.wf(i,k) = 0;     
        end
    end

    % Normalize weights
    pf.wf(:,k) = pf.wf(:,k)./sum(pf.wf(:,k));
    
    %% Resampling
    % Calculate number of effective particles with eq (30)
    Neff = 1/sum(pf.wf(:,k).^2);
    pf.Neff = Neff;

    % Set resampling threshold at 80% of particle count
    resample_percentage = .8;
    Nt = resample_percentage*pf.N;

    % Resample if Neff is less than 80% of N, don't resample last step
    if Neff < Nt && k~=Tp
        [idx, pf.wf(:,k)] = resample(pf.wf(:,k));
        pf.particles(:,:,k) = pf.particles(:,idx,k);
    end
    
    % Extract state estimate
    xh = pf.particles(:,:,k) * pf.wf(:,k);
end

function [idx,wk] = resample(wk)
% Function performs multinomial resampling with replacement of particle set
% using filtering weights
    Ns = length(wk);
    wk = wk./sum(wk);
    with_replacement = true;
    idx = randsample(1:Ns, Ns, with_replacement, wk);
    wk = repmat(1/Ns, 1, Ns);
end