%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function takes the reference trajectory for Agents 2/3 in the
% formation and adds a creates an on-board radar signature emitter. The 
% radar signature emitter from Agents 2/3 is directed towards Agent 1, so
% that Agent 1 can measure the range of Agents 2/3 relative to itself. The
% range measurements using the radar on-board Agent 1 are proccessed in the
% rangeMeasurementRadar.m file.
%
% The signal emissions from Agents 2/3 generated below are independent of 
% the sensing processes conducted on-board of Agent 1.
% 
% Author: Malav Naik
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [emittedSignal] = agentEmitter(refTraj)
