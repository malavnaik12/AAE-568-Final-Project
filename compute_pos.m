%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function takes the state estimate of Agent 1, the range measurement
% between Agent 1 and other agents in the formation and the range 
% measurement's covariance matrix to detemine the position measurement 
% for Agents in the formation. 
% 
% Author: Malav Naik
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [pos_meas_n, meas_Cov_n, prev_state_est] = compute_pos(ekf, rel_vec, rel_Cov, prev_state_1, dt, drop_percent, est_est_flag, rand_num, tree_flag, signal)

curr_pos_1 = ekf.State(5:7);
prev_pos_1 = prev_state_1(1:3);
prev_vel_1 = prev_state_1(4:6);
if ~tree_flag
    [agent1State_est_est, prev_state_est] = stateComputeAgent1(prev_pos_1, prev_vel_1, dt, curr_pos_1, drop_percent, est_est_flag, rand_num);
else
    [agent1State_est_est, prev_state_est] = trees_analysis(prev_pos_1,prev_vel_1,dt,curr_pos_1,drop_percent,signal,est_est_flag);
end

if ~isnan(agent1State_est_est)
    agent1Pos_est_est(1:3,1) = agent1State_est_est(1:3);

    pos_meas_n = agent1Pos_est_est + rel_vec;

    agent1Pos_est_est_Cov = ekf.StateCovariance;
    meas_Cov_n = rel_Cov + agent1Pos_est_est_Cov(5:7,5:7); 
else
    pos_meas_n = NaN;
    meas_Cov_n = NaN;
end

end



