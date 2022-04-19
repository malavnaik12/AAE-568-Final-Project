clc

close all
clear

%% Initialization
setup_agents;

%% Main Simulation Loop

% Loop setup - |trajData| has about 142 seconds of recorded data.
secondsToSimulate = 50; % simulate about 50 seconds
numsamples = secondsToSimulate*fs_imu;

loopBound = floor(numsamples);
loopBound = floor(loopBound/fs_imu)*fs_imu; % ensure enough IMU Samples

% Log data for final metric computation.
pqorient_1 = quaternion.zeros(loopBound,1);
pqpos_1 = zeros(loopBound,3);
pqorient_2 = quaternion.zeros(loopBound,1);
pqpos_2 = zeros(loopBound,3);
pqorient_3 = quaternion.zeros(loopBound,1);
pqpos_3 = zeros(loopBound,3);

fcnt = 1;

while(fcnt <=loopBound)
    % |predict| loop at IMU update frequency.
    for ff=1:fs_imu
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
        pqorient_1(fcnt) = fusedOrient_1;
        pqpos_1(fcnt,:) = fusedPos_1;
        pqorient_2(fcnt) = fusedOrient_2;
        pqpos_2(fcnt,:) = fusedPos_2;
        pqorient_3(fcnt) = fusedOrient_3;
        pqpos_3(fcnt,:) = fusedPos_3;
        
        fcnt = fcnt + 1;
    end
    
    % This next step happens at the GPS sample rate.
    % Simulate the GPS output based on the current pose.
    [lla_1, gpsvel_1] = gps(trajPos_1(fcnt,:), trajVel_1(fcnt,:) );
    
    % Correct the filter states based on the GPS data and magnetic
    % field measurements.
    fusegps(ekf_1, lla_1, Rpos, gpsvel_1, Rvel);
    
    fusemag(ekf_1, mag_1, Rmag);
    fusemag(ekf_2, mag_2, Rmag);
    fusemag(ekf_3, mag_3, Rmag);
 
end






