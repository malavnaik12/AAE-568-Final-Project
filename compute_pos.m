%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function takes the state estimate of Agent 1, the range measurement
% between Agent 1 and other agents in the formation and the range 
% measurement's covariance matrix to detemine the position measurement 
% for Agents in the formation. 
% 
% Author: Malav Naik
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [pos_meas_n, meas_Cov_n] = compute_pos(ekf, rel_vec, rel_Cov)