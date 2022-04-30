%% Plotting Code
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

figure(2)
title('Total Position Estimate Error vs Time')
hold on
grid minor
plot((1:loopBound)/fs_imu, sqrt(posd_1(:,1,1).^2 + posd_1(:,2,1).^2 + posd_1(:,3,1).^2),'b')
plot((1:loopBound)/fs_imu, sqrt(posd_2(:,1,1).^2 + posd_2(:,2,1).^2 + posd_2(:,3,1).^2),'r')
plot((1:loopBound)/fs_imu, sqrt(posd_3(:,1,1).^2 + posd_3(:,2,1).^2 + posd_3(:,3,1).^2),'Color',[0,128,0]/255)
xlabel('Time [sec]');ylabel('Total Position Error [m]')
legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northeast')
ylim([0 5])


figure(3)
hold on; grid minor
title({'Root Mean Square Error (RMSE) vs. Dropout Percentage',''})
if (numel(size(errorTotal)) == 2)
    plot(drop_range*100, RMSE_PA,'b-','LineWidth',1.2)
    plot(drop_range*100, errorTotal(:,2,1),'r-','LineWidth',1.2)
    plot(drop_range*100, errorTotal(:,3,1),'LineStyle','-','LineWidth',1.2,'Color',[0,128,0]/255)
    set(gca,'YScale','log')
    xlabel('Dropout Percentage [%]');
    ylabel('Total Root Mean Square Error [m]')
    legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northwest')
elseif (numel(size(errorTotal)) == 3)
    plot(drop_range*100, RMSE_PA,'b-','LineWidth',1.2)
    plot(drop_range*100, errorTotal(:,2,2),'r-','LineWidth',1.2)
    plot(drop_range*100, errorTotal(:,3,2),'LineStyle','-','LineWidth',1.2,'Color',[0,128,0]/255)
    xlabel('Dropout Percentage [%]');
    set(gca,'YScale','log')
    plot(drop_range*100, errorTotal(:,2,1),'r--','LineWidth',1.2)
    plot(drop_range*100, errorTotal(:,3,1),'LineStyle','--','LineWidth',1.2,'Color',[0,128,0]/255)
    ylabel('Total Root Mean Square Error [m] (Log Scale)')
    legend('Primary Agent', 'Secondary Agent 1 - Case 1', 'Secondary Agent 2 - Case 1','Secondary Agent 1 - Case 2','Secondary Agent 2 - Case 2','Location','northwest')
elseif (treeflag == 1)
    plot(drop_range*100, RMSE_PA,'b-','LineWidth',1.2)
    plot(drop_range*100, errorTotal(:,2,1),'r-','LineWidth',1.2)
    plot(drop_range*100, errorTotal(:,3,1),'LineStyle','-','LineWidth',1.2,'Color',[0,128,0]/255)
    xlabel('Dropout Percentage [%]');
    set(gca,'YScale','log')
    ylabel('Total Root Mean Square Error [m] (Log Scale)')
    legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2')
end

figure(4)
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
xlim([35 50]);

subplot(4,1,2)
hold on
grid minor
plot((1:loopBound)/fs_imu, posd_1(:,2,1),'b')
plot((1:loopBound)/fs_imu, posd_2(:,2,1),'r')
plot((1:loopBound)/fs_imu, posd_3(:,2,1),'Color',[0,128,0]/255)
plot((1:loopBound)/fs_imu,zeros(1,length(1:loopBound)),'k--')
xlabel('Time [sec]');ylabel('Y Position Error [m]')
legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northeastoutside')  
xlim([35 50]);

subplot(4,1,3)
hold on
grid minor
plot((1:loopBound)/fs_imu, posd_1(:,3,1),'b')
plot((1:loopBound)/fs_imu, posd_2(:,3,1),'r')
plot((1:loopBound)/fs_imu, posd_3(:,3,1),'Color',[0,128,0]/255)
plot((1:loopBound)/fs_imu,zeros(1,length(1:loopBound)),'k--')
xlabel('Time [sec]');ylabel('Z Position Error [m]')
legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northeastoutside')  
xlim([35 50]);

subplot(4,1,4)
hold on
grid minor
plot((1:loopBound)/fs_imu, sqrt(posd_1(:,1,1).^2 + posd_1(:,2,1).^2 + posd_1(:,3,1).^2),'b')
plot((1:loopBound)/fs_imu, sqrt(posd_2(:,1,1).^2 + posd_2(:,2,1).^2 + posd_2(:,3,1).^2),'r')
plot((1:loopBound)/fs_imu, sqrt(posd_3(:,1,1).^2 + posd_3(:,2,1).^2 + posd_3(:,3,1).^2),'Color',[0,128,0]/255)
xlabel('Time [sec]');ylabel('Total Position Error [m]')
legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northeastoutside')
xlim([35 50]);

figure(4)
title('Total Position Estimate Error vs Time')
hold on
grid minor
plot((1:loopBound)/fs_imu, sqrt(posd_1(:,1,1).^2 + posd_1(:,2,1).^2 + posd_1(:,3,1).^2),'b')
plot((1:loopBound)/fs_imu, sqrt(posd_2(:,1,1).^2 + posd_2(:,2,1).^2 + posd_2(:,3,1).^2),'r')
plot((1:loopBound)/fs_imu, sqrt(posd_3(:,1,1).^2 + posd_3(:,2,1).^2 + posd_3(:,3,1).^2),'Color',[0,128,0]/255)
xlabel('Time [sec]');ylabel('Total Position Error [m]')
legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northeastoutside')


figure(5);
title('Total Position Estimate Error vs Time')
subplot(1,2,1)
hold on
grid minor
plot((1:loopBound)/fs_imu, sqrt(posd_1(:,1,1).^2 + posd_1(:,2,1).^2 + posd_1(:,3,1).^2),'b')
plot((1:loopBound)/fs_imu, sqrt(posd_2(:,1,1).^2 + posd_2(:,2,1).^2 + posd_2(:,3,1).^2),'r')
plot((1:loopBound)/fs_imu, sqrt(posd_3(:,1,1).^2 + posd_3(:,2,1).^2 + posd_3(:,3,1).^2),'Color',[0,128,0]/255)
xlabel('Time [sec]');ylabel('Total Position Error [m]')
legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northeast')
subplot(1,2,2)
hold on
grid minor
plot((1:loopBound)/fs_imu, sqrt(posd_1(:,1,1).^2 + posd_1(:,2,1).^2 + posd_1(:,3,1).^2),'b')
plot((1:loopBound)/fs_imu, sqrt(posd_2(:,1,1).^2 + posd_2(:,2,1).^2 + posd_2(:,3,1).^2),'r')
plot((1:loopBound)/fs_imu, sqrt(posd_3(:,1,1).^2 + posd_3(:,2,1).^2 + posd_3(:,3,1).^2),'Color',[0,128,0]/255)
xlabel('Time [sec]');ylabel('Total Position Error [m]')
legend('Primary Agent', 'Secondary Agent 1', 'Secondary Agent 2','Location','northeast')
xlim([35 50]);

