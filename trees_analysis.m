function [pos_est_est] = trees_analysis(prev_pos_1,prev_vel_1,dt,curr_pos_1,drop_percent,num_sec_trees,num_sec_clear)

rng(55);
d_signal = rand(1,num_sec_trees); %disturbing signal simulates signal passing through obstabcles 
s_signal = zeros(1,num_sec_clear); %stable signal simulates signal perfectly transimitted
signal = [d_signal s_signal];

if any(signal < drop_percent) 
    pos_est_est_1 = prev_pos_1 + prev_vel_1*dt;

else 
     pos_est_est_1 = curr_pos_1; %pose(ekf_1)
end

end
