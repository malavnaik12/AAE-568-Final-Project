%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function takes the reference position for Agent 1 in the
% formation and adds an on-board radar. The radar is then used to
% measure the range of Agents 2/3 relative to Agent 1.
% 
% Author: Malav Naik
% Command for Example: openExample('fusion/ModelAirTrafficControlTowerScanningExample')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [measurements, covariance] = rangeMeasurementRadar(EstAgent1,EstAgent2,EstAgent3,simTime)
poseAgent1 = EstAgent1(:,1); velAgent1 = EstAgent1(:,2);
poseAgent2 = EstAgent2(:,1); velAgent2 = EstAgent2(:,2);
poseAgent3 = EstAgent3(:,1); velAgent3 = EstAgent3(:,2);

tgt1 = struct('PlatformID',1,'Position',poseAgent2','Velocity',velAgent2');
tgt2 = struct('PlatformID',2,'Position',poseAgent3','Velocity',velAgent3');

rpm = 12.5;
fov = [1.4;5]; % [azimuth; elevation]
scanrate = rpm*360/60;  % deg/s
updaterate = scanrate/fov(1); % Hz

sensor = fusionRadarSensor(1,'Rotator', 'UpdateRate',updaterate, ...
    'MountingLocation',poseAgent1', 'MaxAzimuthScanRate',scanrate, ...
    'FieldOfView',fov, 'AzimuthResolution',fov(1));
% simTime = 0;
detBuffer = {};
while true
    [dets,numDets,config] = sensor([tgt1 tgt2],simTime);
    detBuffer = [detBuffer; dets]; %#ok<AGROW>

    % Is full scan complete?
    if config.IsScanDone
        break % yes
    end
    simTime = simTime + 1/sensor.UpdateRate;
end

radarPosition = poseAgent1;
tgtPositions = [poseAgent2; poseAgent3];

% The example uses a similar structure to that found below to extra data
% from the detBuffer cell, which is developed in the above while loop by
% invoking the fusionRadarSensor. I am not really sure about what the
% results in detBuffer are supposed to represent tbh. Also detBuffer has a 
% matrix titled Measurement Noise, which I assume are the noise
% characteristics for a given measurement but again, not sure about that.

% The structure of detPosError is as follows: first three rows represent 
% measurement noise for the measurement in the first row of detPos, next 
% three rows represent noise for the measurement in the second row of 
% detPos, and so on and so forth.
if ~isempty(detBuffer)
    detPos = cellfun(@(d)d.Measurement(1:3),detBuffer,'UniformOutput',false);
    detPos = cell2mat(detPos')';
    detPosError = cellfun(@(d)d.MeasurementNoise(:,:),detBuffer,'UniformOutput',false);
    detPosError = cell2mat(detPosError')';
end

measurements = detPos - radarPosition;
% Covariance model is missing, the measurement noise is given by
% detPosError, but need to figure out how to use it.
covariance = 0; % Placeholder variable