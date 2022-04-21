%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function takes the EKF object for Agent n and Agent 1 as input. 
% Processes the relative position measurement from radar with the estimated
% position of Agent 1 to compute the measured position of Agent n. This
% position measurement of Agent n is fused into the EKF estimate for Agent
% n to correct for the drift of the IMU sensors
% 
% Author: Nathan Gurgens
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function fuserange(ekf_1, ekf_n, rel_vec, rel_Cov)

% Compute the position of Agent n given the relative position measurement
% between Agent 1 and Agent n and the estimated position of Agent 1

[pos_meas_n, meas_Cov_n] = compute_pos_n(ekf_1, rel_vec, rel_Cov);

range_correction(ekf_n, pos_meas_n, @rangeMeasFcn, meas_Cov_n, @rangeMeasJacobianFcn)

end


function range_correction(ekf, z, measFcn, measCov, measJacobianFcn)

x_est = ekf.State;
h = measFcn(x_est);
H = measJacobianFcn();
P_est = ekf.StateCovariance;

K = P_est*H.'/(H*P_est*(H.') + measCov);
x_est = x_est + K*(z - h);
P_est = P_est - K*H*P_est;

x_est = repairQuaternion(ekf,x_est); 

ekf.State = x_est;
ekf.StateCovariance = P_est;

end

function z = rangeMeasFcn(x)

% Getting only 3 positional terms from state x

z = [x(5) x(6) x(7)].';

end

function dhdx = rangeMeasJacobianFcn()

dhdx = [0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
    
end

%Copied from MATLAB's BasicEKF.m to avoid syntax issues
function x = repairQuaternion(obj, x)
    % Normalize quaternion and enforce a positive angle of
    % rotation. Avoid constructing a quaternion here. Just do the
    % math inline.

    % Used cached OrientationIdx
    idx = obj.OrientationIdx;
    qparts = x(idx);
    n = sqrt(sum(qparts.^2));
    qparts = qparts./n;
    if qparts(1) < 0
        x(idx) = -qparts;
    else
        x(idx) = qparts;
    end
end





