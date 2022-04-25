function [pos_est_est_1] = trees_analysis(prev_pos_1,prev_vel_1,dt,curr_pos_1,drop_percent,signal,est_est_flag)

% rng(55);
% d_signal = rand(1,num_sec_trees); %disturbing signal simulates signal passing through obstabcles 
% s_signal = zeros(1,num_sec_clear); %stable signal simulates signal perfectly transimitted
% signal = [d_signal s_signal];
if est_est_flag == 1
    if any(signal < drop_percent) 
        pos_est_est_1 = prev_pos_1 + prev_vel_1*dt;
    else 
         pos_est_est_1 = curr_pos_1; %pose(ekf_1)
    end
else
    if any(signal < drop_percent) 
        pos_est_est_1 = NaN;
    else 
         pos_est_est_1 = curr_pos_1; 
    end
end
end
