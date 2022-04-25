

for i = 1:length(drop_range)
    posd_1(:,:,i) = pqpos_1(1:loopBound,:,i) - trajPos_1( 1:loopBound, :);
    posd_2(:,:,i) = pqpos_2(1:loopBound,:,i) - trajPos_2( 1:loopBound, :);
    posd_3(:,:,i) = pqpos_3(1:loopBound,:,i) - trajPos_3( 1:loopBound, :);

%     error_sum(:,:,i) = (posd_1(:,:,i).^2 + posd_2(:,:,i).^2 + posd_3(:,:,i).^2);
%     errorTotal(i,:) = rms(error_sum(:,:,i));
    error_sum_1(:,i) = sqrt(posd_1(:,1,i).^2 + posd_1(:,2,i).^2 + posd_1(:,3,i).^2);
    errorTotal_1(i) = rms(error_sum_1(:,i));
    error_sum_2(:,i) = sqrt(posd_2(:,1,i).^2 + posd_2(:,2,i).^2 + posd_2(:,3,i).^2);
    errorTotal_2(i) = rms(error_sum_2(:,i));
    error_sum_3(:,i) = sqrt(posd_3(:,1,i).^2 + posd_3(:,2,i).^2 + posd_3(:,3,i).^2);
    errorTotal_3(i) = rms(error_sum_3(:,i));
    
    msep_1(:,i) = sqrt(mean(posd_1(:,:,i).^2));
    msep_2(:,i) = sqrt(mean(posd_2(:,:,i).^2));
    msep_3(:,i) = sqrt(mean(posd_3(:,:,i).^2));

end

msep_x = [msep_1(1,:); msep_2(1,:); msep_3(1,:)];
msep_y = [msep_1(2,:); msep_2(2,:); msep_3(2,:)];
msep_z = [msep_1(3,:); msep_2(3,:); msep_3(3,:)];


% quatd_1 = rad2deg(dist(pqorient_1(1:loopBound), trajOrient_1(1:loopBound)) );
% quatd_2 = rad2deg(dist(pqorient_2(1:loopBound), trajOrient_2(1:loopBound)) );

% Display RMS errors in the command window.
% fprintf('\n\nEnd-to-End Simulation Position RMS Error\n');

% for jj = 1:length(drop_range)
%     fprintf('\tX1: %.2f , Y1: %.2f, Z1: %.2f   (meters)\n\n',msep_1(1), ...
%         msep_1(2), msep_1(3));
% 
%     fprintf('\tX2: %.2f , Y2: %.2f, Z2: %.2f   (meters)\n\n',msep_2(1), ...
%         msep_2(2), msep_2(3));
% 
%     fprintf('\tX3: %.2f , Y3: %.2f, Z3: %.2f   (meters)\n\n',msep_3(1), ...
%         msep_3(2), msep_3(3));
% end
% % Display RMS errors in the command window.
% fprintf('\n\nEnd-to-End Simulation Position RMS Error\n');
% 
% fprintf('\tX1: %.2f , Y1: %.2f, Z1: %.2f   (meters)\n\n',msep_1(1), ...
%     msep_1(2), msep_1(3));
% 
% fprintf('\tX2: %.2f , Y2: %.2f, Z2: %.2f   (meters)\n\n',msep_2(1), ...
%     msep_2(2), msep_2(3));
% 
% fprintf('\tX3: %.2f , Y3: %.2f, Z3: %.2f   (meters)\n\n',msep_3(1), ...
%     msep_3(2), msep_3(3));

% fprintf('End-to-End Quaternion Distance RMS Error (degrees) \n');
% fprintf('\t%.2f (degrees)\n\n', sqrt(mean(quatd_1.^2)));
% fprintf('\t%.2f (degrees)\n\n', sqrt(mean(quatd_2.^2)));

figure(1)

subplot(4,1,1)
title('Estimate Error vs Time')
hold on
grid minor
plot((1:loopBound)/fs_imu, posd_1(:,1,1),'b')
plot((1:loopBound)/fs_imu, posd_2(:,1,1),'r')
plot((1:loopBound)/fs_imu, posd_3(:,1,1),'Color',[0,128,0]/255)
plot((1:loopBound)/fs_imu,zeros(1,length(1:loopBound)),'k--')
xlabel('Time [sec]');ylabel('X Position Error [m]')
legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northeastoutside')  

subplot(4,1,2)
hold on
grid minor
plot((1:loopBound)/fs_imu, posd_1(:,2,1),'b')
plot((1:loopBound)/fs_imu, posd_2(:,2,1),'r')
plot((1:loopBound)/fs_imu, posd_3(:,2,1),'Color',[0,128,0]/255)
plot((1:loopBound)/fs_imu,zeros(1,length(1:loopBound)),'k--')
xlabel('Time [sec]');ylabel('Y Position Error [m]')
legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northeastoutside')  

subplot(4,1,3)
hold on
grid minor
plot((1:loopBound)/fs_imu, posd_1(:,3,1),'b')
plot((1:loopBound)/fs_imu, posd_2(:,3,1),'r')
plot((1:loopBound)/fs_imu, posd_3(:,3,1),'Color',[0,128,0]/255)
plot((1:loopBound)/fs_imu,zeros(1,length(1:loopBound)),'k--')
xlabel('Time [sec]');ylabel('Z Position Error [m]')
legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northeastoutside')  

subplot(4,1,4)
hold on
grid minor
plot((1:loopBound)/fs_imu, sqrt(posd_1(:,1,1).^2 + posd_1(:,2,1).^2 + posd_1(:,3,1).^2),'b')
plot((1:loopBound)/fs_imu, sqrt(posd_2(:,1,1).^2 + posd_2(:,2,1).^2 + posd_2(:,3,1).^2),'r')
plot((1:loopBound)/fs_imu, sqrt(posd_3(:,1,1).^2 + posd_3(:,2,1).^2 + posd_3(:,3,1).^2),'Color',[0,128,0]/255)

xlabel('Time [sec]');ylabel('Total Position Error [m]')
legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northeastoutside') 







