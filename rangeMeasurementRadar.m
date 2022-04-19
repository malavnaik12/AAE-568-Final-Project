%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function takes the reference trajectory for Agent 1 in the
% formation and adds a creates an on-board radar. The radar is then used to
% measure the range of Agents 2/3 relative to Agent 1. The emitted signals
% originating from Agents 2/3, which are then sensed by this radar object,
% are created using the function within agentEmitter.m
%
% The signal emissions are independent of the sensing processes conducted
% below on-board of Agent 1.
% 
% Author: Malav Naik
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [measurements] = rangeMeasurementRadar(refTraj)
