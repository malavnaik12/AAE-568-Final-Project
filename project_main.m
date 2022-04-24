clc
close all
clear

%% Initialization
setup_agents;

%% Main Simulation Loop Setup
% Loop setup - |trajData| has about 142 seconds of recorded data.
secondsToSimulate = 100; % simulate about 50 seconds
numsamples = secondsToSimulate*fs_imu;
rand_vec = rand(secondsToSimulate*2,1);

loopBound = floor(numsamples);
loopBound = floor(loopBound/fs_imu)*fs_imu; % ensure enough IMU Samples

%% Analysis Setup
drop_range = 1;
est_est_range = 0;

tree_flag = 1;
tree_range = 0.5;

%% Main Simulation Loop

% Log data for final metric computation.
pqorient_1 = quaternion.zeros(loopBound,1,length(drop_range)*length(tree_range));
pqpos_1 = zeros(loopBound,3,length(drop_range)*length(tree_range));
pqorient_2 = quaternion.zeros(loopBound,1,length(drop_range)*length(tree_range));
pqpos_2 = zeros(loopBound,3,length(drop_range)*length(tree_range));
pqorient_3 = quaternion.zeros(loopBound,1,length(drop_range)*length(tree_range));
pqpos_3 = zeros(loopBound,3,length(drop_range)*length(tree_range));

prev_state_1_2 = ekf_1.State;
prev_state_1_3 = ekf_1.State;

for est_est_flag = est_est_range
    for tree_percent = tree_range
        tree_percent
        drop_count = 1;

        num_sec_trees = tree_percent*secondsToSimulate;
        num_sec_clear = (1-tree_percent)*secondsToSimulate;
        d_signal = rand(1,num_sec_trees*2); %disturbing signal simulates signal passing through obstabcles 
        s_signal = ones(1,num_sec_clear*2); %stable signal simulates signal perfectly transimitted
        signal = [d_signal s_signal];
        
        for drop_percent = drop_range%(0:100)/100
            drop_percent
            fcnt = 1;
            sec_count = 1;
            while(fcnt <=loopBound)
                for ff=1:fs_imu
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % |predict| loop at IMU update frequency.
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                    % Simulate the IMU data from the current pose.
                    [accel_1, gyro_1, mag_1] = imu_1(trajAcc_1(fcnt,:), trajAngVel_1(fcnt, :),trajOrient_1(fcnt));
                    [accel_2, gyro_2, mag_2] = imu_2(trajAcc_2(fcnt,:), trajAngVel_2(fcnt, :),trajOrient_2(fcnt));
                    [accel_3, gyro_3, mag_3] = imu_3(trajAcc_3(fcnt,:), trajAngVel_3(fcnt, :),trajOrient_3(fcnt));

                    % Use the |predict| method to estimate the filter state based
                    % on the simulated accelerometer and gyroscope signals.
                    predict(ekf_1, accel_1, gyro_1);
                    predict(ekf_2, accel_2, gyro_2);
                    predict(ekf_3, accel_3, gyro_3);

                    % Acquire the current estimate of the filter states.
                    [fusedPos_1, fusedOrient_1] = pose(ekf_1);
                    [fusedPos_2, fusedOrient_2] = pose(ekf_2);
                    [fusedPos_3, fusedOrient_3] = pose(ekf_3);

                    % Save the position and orientation for post processing.
                    pqorient_1(fcnt,drop_count) = fusedOrient_1;
                    pqpos_1(fcnt,:,drop_count) = fusedPos_1;
                    pqorient_2(fcnt,drop_count) = fusedOrient_2;
                    pqpos_2(fcnt,:,drop_count) = fusedPos_2;
                    pqorient_3(fcnt,drop_count) = fusedOrient_3;
                    pqpos_3(fcnt,:,drop_count) = fusedPos_3;

                    fcnt = fcnt + 1;
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % This next step happens at the GPS sample rate.
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                % Simulate the GPS output based on the current pose.
                [lla_1, gpsvel_1] = gps(trajPos_1(fcnt,:), trajVel_1(fcnt,:) );

                % Simulate the radar measurements based on the current pose
                [rel_pos_NED_1_to_2, rel_Cov_2] = rangeMeasAddedNoise(ekf_1, trajPos_2(fcnt,:), noiseMean, noiseVar, 2);
                [rel_pos_NED_1_to_3, rel_Cov_3] = rangeMeasAddedNoise(ekf_1, trajPos_3(fcnt,:), noiseMean, noiseVar, 1000);

                % Correct the filter states based on the GPS measurement.
                fusegps(ekf_1, lla_1, Rpos, gpsvel_1, Rvel);

                % Correct the filter states based on the relative position measurement.
                fuserange(ekf_1, ekf_2, rel_pos_NED_1_to_2, rel_Cov_2, prev_state_1_2, 1/fs_gps, drop_percent, est_est_flag, rand_vec(sec_count), tree_flag, signal(sec_count))
                fuserange(ekf_1, ekf_3, rel_pos_NED_1_to_3, rel_Cov_3, prev_state_1_3, 1/fs_gps, drop_percent, est_est_flag, rand_vec(sec_count+1), tree_flag, signal(sec_count+1));

                % Correct the filter states based on the magnetic field measurement.
                fusemag(ekf_1, mag_1, Rmag);
                fusemag(ekf_2, mag_2, Rmag);
                fusemag(ekf_3, mag_3, Rmag);

                if rand_vec(sec_count) > drop_percent
                    prev_state_1_2 = ekf_1.State;
                end

                if rand_vec(sec_count+1) > drop_percent
                    prev_state_1_3 = ekf_1.State;
                end

                sec_count = sec_count+2;
            end
            drop_count = drop_count+1;  
        end
    end
end

plotting



