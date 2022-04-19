%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function takes the EKF object for Agent n and Agent 1 as input. 
% Processes the relative position measurement from radar with the estimated
% position of Agent 1 to compute the measured position of Agent n. This
% position measurement of Agent n is fused into the EKF estimate for Agent
% n to correct for the drift of the IMU sensors
% 
% Author: Nathan Gurgens
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function fuserange(ekf_1, ekf_n, rel_vec, rel_Cov)

% Compute the position of Agent n given the relative position measurement
% between Agent 1 and Agent n and the estimated position of Agent 1

% @MALAV [pos_meas_n, meas_cov_n] = compute_pos_n(ekf_1, rel_vec, rel_Cov)










