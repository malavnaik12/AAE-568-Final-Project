
clf,close all
for i = 1:length(drop_range)
    posd_1(:,:,i) = pqpos_1(1:loopBound,:,i) - trajPos_1( 1:loopBound, :);
    posd_2(:,:,i) = pqpos_2(1:loopBound,:,i) - trajPos_2( 1:loopBound, :);
    posd_3(:,:,i) = pqpos_3(1:loopBound,:,i) - trajPos_3( 1:loopBound, :);

%     error_sum(:,:,i) = (posd_1(:,:,i).^2 + posd_2(:,:,i).^2 + posd_3(:,:,i).^2);
%     errorTotal(i,:) = rms(error_sum(:,:,i));
    error_sum_1(:,i) = (posd_1(:,1,i).^2 + posd_1(:,2,i).^2 + posd_1(:,3,i).^2);
    errorTotal_1(i) = rms(error_sum_1(:,i));
    error_sum_2(:,i) = (posd_2(:,1,i).^2 + posd_2(:,2,i).^2 + posd_2(:,3,i).^2);
    errorTotal_2(i) = rms(error_sum_2(:,i));
    error_sum_3(:,i) = (posd_3(:,1,i).^2 + posd_3(:,2,i).^2 + posd_3(:,3,i).^2);
    errorTotal_3(i) = rms(error_sum_3(:,i));
    
    msep_1(:,i) = sqrt(mean(posd_1(:,:,i).^2)/numel(posd_1(:,:,i)));
    msep_2(:,i) = sqrt(mean(posd_2(:,:,i).^2)/numel(posd_2(:,:,i)));
    msep_3(:,i) = sqrt(mean(posd_3(:,:,i).^2)/numel(posd_2(:,:,i)));
%     msep_total(:,i) = sqrt((mean(posd_1(:,:,i).^2) + mean(posd_2(:,:,i).^2) + mean(posd_3(:,:,i).^2))...
%                         /(numel(posd_1(:,:,i)) + numel(posd_2(:,:,i)) + numel(posd_3(:,:,i))));

end

msep_x = [msep_1(1,:); msep_2(1,:); msep_3(1,:)];
msep_y = [msep_1(2,:); msep_2(2,:); msep_3(2,:)];
msep_z = [msep_1(3,:); msep_2(3,:); msep_3(3,:)];

quatd_1 = rad2deg(dist(pqorient_1(1:loopBound), trajOrient_1(1:loopBound)) );
quatd_2 = rad2deg(dist(pqorient_2(1:loopBound), trajOrient_2(1:loopBound)) );

% Display RMS errors in the command window.
fprintf('\n\nEnd-to-End Simulation Position RMS Error\n');

for jj = 1:length(drop_range)
    fprintf('\tX1: %.2f , Y1: %.2f, Z1: %.2f   (meters)\n\n',msep_1(1), ...
        msep_1(2), msep_1(3));

    fprintf('\tX2: %.2f , Y2: %.2f, Z2: %.2f   (meters)\n\n',msep_2(1), ...
        msep_2(2), msep_2(3));

    fprintf('\tX3: %.2f , Y3: %.2f, Z3: %.2f   (meters)\n\n',msep_3(1), ...
        msep_3(2), msep_3(3));
end
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
title('Error Comparison Between Fusion Filter and IMU Only Filter')
hold on
grid minor
plot(1:loopBound, posd_1(:,1,1),'b')
plot(1:loopBound, posd_2(:,1,1),'r')
plot(1:loopBound, posd_3(:,1,1),'Color',[0,128,0]/255)
xlabel('k');ylabel('X Position Error [m]')
legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northeastoutside')  

subplot(4,1,2)
hold on
grid minor
plot(1:loopBound, posd_1(:,2,1),'b')
plot(1:loopBound, posd_2(:,2,1),'r')
plot(1:loopBound, posd_3(:,2,1),'Color',[0,128,0]/255)
xlabel('k');ylabel('Y Position Error [m]')
legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northeastoutside')  

subplot(4,1,3)
hold on
grid minor
plot(1:loopBound, posd_1(:,3,1),'b')
plot(1:loopBound, posd_2(:,3,1),'r')
plot(1:loopBound, posd_3(:,3,1),'Color',[0,128,0]/255)
xlabel('k');ylabel('Z Position Error [m]')
legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northeastoutside')  

subplot(4,1,4)
hold on
grid minor
plot(1:loopBound, sqrt(posd_1(:,1,1).^2 + posd_1(:,2,1).^2 + posd_1(:,3,1).^2),'b')
plot(1:loopBound, sqrt(posd_2(:,1,1).^2 + posd_2(:,2,1).^2 + posd_2(:,3,1).^2),'r')
plot(1:loopBound, sqrt(posd_3(:,1,1).^2 + posd_3(:,2,1).^2 + posd_3(:,3,1).^2),'Color',[0,128,0]/255)

xlabel('k');ylabel('Total Position Error [m]')
legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northeastoutside') 

figure(2)
sgtitle('RMS Error Comparison Between Various Drop Percents')
subplot(4,1,1)
hold on; grid minor
plot(drop_range, msep_x(1,:),'b')
plot(drop_range, msep_y(1,:),'r')
plot(drop_range, msep_z(1,:),'Color',[0,128,0]/255)
legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northeastoutside')
xlabel('Dropout Percent, %'); ylabel('RMS Error, X Position [m]')

subplot(4,1,2)
hold on; grid minor
plot(drop_range, msep_x(2,:),'b')
plot(drop_range, msep_y(2,:),'r')
plot(drop_range, msep_z(2,:),'Color',[0,128,0]/255)
legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northeastoutside')
xlabel('Dropout Percent, %'); ylabel('RMS Error, Y Position [m]')

subplot(4,1,3)
hold on; grid minor
plot(drop_range, msep_x(3,:),'b')
plot(drop_range, msep_y(3,:),'r')
plot(drop_range, msep_z(3,:),'Color',[0,128,0]/255)
legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northeastoutside')
xlabel('Dropout Percent, %'); ylabel('RMS Error, Z Position [m]')

subplot(4,1,4)
hold on; grid minor
plot(drop_range, errorTotal_1,'b')
plot(drop_range, errorTotal_2,'r')
plot(drop_range, errorTotal_3,'Color',[0,128,0]/255)
legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northeastoutside')
xlabel('Dropout Percent, %'); ylabel('RMS Error, Total [m]')



