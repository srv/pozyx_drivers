
function plot_uwb(logFile)

data = load(['/home/xisco/expUWB/position',logFile, '.txt']);
%data2 = load(['/home/xisco/expUWB/filt_ICP_',logFile, '.txt']);

figure(1)
hold on;
plot3(data(:,2),data(:,3),data(:,4), 'LineWidth', 2);
plot3(data(1,2),data(1,3),data(1,4), 'xg', 'LineWidth', 2);
plot3(data(end,2),data(end,3),data(end,4), 'or', 'LineWidth', 2);

%plot3(data2(:,2),data2(:,3),data2(:,4), 'm', 'LineWidth', 2);
%plot3(data2(1,2),data2(1,3),data2(1,4), 'xg', 'LineWidth', 2);
%plot3(data2(end,2),data2(end,3),data2(end,4), 'or', 'LineWidth', 2);

grid on;
axis equal;