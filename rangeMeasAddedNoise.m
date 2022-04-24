%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function takes the reference trajectories for Agents in the
% formation and adds white Gaussian noise to simulate the measured position
% information of the agents. The simulated measurements of the agents are
% then used to calculate the range vector between two agents for IMU/GPS 
% fusion among the Agents within the formation.
% 
% Author: Malav Naik
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [rangeMeas,covariance_n] = rangeMeasAddedNoise(ekf, trajPos_n, mean, variance, seedInitializer)
% rng(agentID);
% for ii = 1:numel(trajPos_1(1,:))
%     rng(ii);
%     noise_1(ii,1) = mean + sqrt(gps_variance)*randn(size(trajPos_1(ii))); % Variance here needs to be GPS variance
%     trajPos1_meas(ii,1) = trajPos_1(ii) + noise_1(ii);
% end

trajPos1_meas = pose(ekf).';

for jj = 1:numel(trajPos_n(1,:))
    rng(jj+seedInitializer);
    noise_n(jj,1) = mean + sqrt(variance(jj))*randn(size(trajPos_n(jj)));
    trajPos_n_meas(jj,1) = trajPos_n(jj) + noise_n(jj);
end
% trajPos_n_meas
covariance_n = diag(variance); %*eye(numel(trajPos_n(1,:)));

rangeMeas = trajPos_n_meas - trajPos1_meas;