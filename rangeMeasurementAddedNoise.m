%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function takes the reference trajectories for Agents in the
% formation and adds white Gaussian noise to simulate the measured position
% information of the agents. The simulated measurements of the agents are
% then used for IMU/GPS fusion among the Agents within the formation.
% 
% Author: Malav Naik
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [measurements,covariance] = rangeMeasurementAddedNoise(refTraj, mean, variance)

% measurements = awgn(refTraj,10,'measured');
rng(1);
for ii = 1:numel(refTraj(1,:))
    noise(:,ii) = mean + sqrt(variance)*randn(size(refTraj(:,ii)));
    measurements(:,ii) = refTraj(:,ii) + randn(size(refTraj(:,ii)));
    
    % Covariance model is missing
    covariance = 0; % Placeholder variable
end