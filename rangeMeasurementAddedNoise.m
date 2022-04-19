%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function takes the reference trajectories for Agents in the
% formation and adds white Gaussian noise to simulate the measured position
% information of the agents. The simulated measurements of the agents are
% then used for IMU/GPS fusion among the Agents within the formation.
% 
% Author: Malav Naik
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [measurements] = rangeMeasurementAddedNoise(refTraj)

measurements = awgn(refTraj,10,'measured');