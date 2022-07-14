function [xs, pf] = reweightingsmoother(pf,Z)
% Function implements reweighting particle smoother by calculating
% smoothing weights with equation (33) and generating smoothed trajectory estimate 

% Inputs:
%  pf: Particle filter structure with relevant fields:
%   pf.particles:   particles (nx x N x Tp)
%   pf.wf:          filtering weights (N x Tp)
%   pf.ws:          smoothing weights (N x Tp)
%   pf.N:           particle count
%   pf.k:           smoothing iteration index
%   pf.dt:          discretization time
%  Z:  Structure containing all road information with relevant fields:
%   Z.Q:            process noise covariance (nx x nx)

% Outputs:
%  xs: smoothed state trajectory estimate
%  pf: Particle Filter structure with updated smoothing weights
%   pf.ws:          smoothing weights

    % Extract parameters from inputs
    N = pf.N;
    k = pf.k;
    dt = pf.dt;
    Q = Z.Q;

    % Initialize memory for transition probabilities
    pij = zeros(N);
    tprob = zeros(N);

    % Generate matrix of weighted transition probabilities
    % This loop generates the numerator in sumation in report equation (33)
    for j = 1:N
        tprob(:,j) = mvnpdf(pf.particles(:,:,k+1)',vehicle_model(pf.particles(:,j,k),dt,Q)',Q);
        pij(:,j) = tprob(:,j) * pf.wf(j,k);
    end

    % Sum across rows - denominator of eq (33)
    pijs = sum(pij,2);

    % Perturb zeros for numerical stability
    pijs(pijs==0) = 1e-100;

    % Troubleshooting NaN transition probabilities before normalization
    if sum(any(isnan(pij))) ~= 0
        ;
    end

    % Divide weighted transitions by row sum - quotient of eq (33)
    pij = pij./pijs;

    % Troubleshooting NaN transition probabilities after normalization
    if sum(any(isnan(pij))) ~= 0
        ;
    end

    % Multiply by k+1 smoothing weight (backwards iteration)
    pij = pij.*pf.ws(:,k+1);
    
    % Sum across columns to get vector of ws[k] - final result of eq (33)
    pf.ws(:,k) = sum(pij,1)';

    % Apply smoothing weights to particle set
    xs = pf.particles(:,:,k) * pf.ws(:,k);
end