%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function takes the state estimate of Agent 1, the range measurement
% between Agent 1 and other agents in the formation and the range 
% measurement's covariance matrix to detemine the position measurement 
% for Agents in the formation. 
% 
% Author: Malav Naik
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [pos_meas_n, meas_Cov_n] = compute_pos(ekf, rel_vec, rel_Cov)

% Zhiyao's function: stateComputeAgent1()
[agent1State_est_est, agent1Pos_est_est_Cov] = stateComputeAgent1(ekf,prev_pos_1, prev_vel_1, dt, curr_pos_1, drop_percent, est_est_flag);

agent1Pos_est_est = agent1State_est_est(1:3,1);

pos_meas_n = agent1Pos_est_est + rel_vec;

meas_Cov_n = rel_Cov + agent1Pos_est_est_Cov; 
            % Not sure about agent1Pos_est_est_Cov above, we can discuss on
            % Saturday