% filename = 'teleop.txt'; % specify the filename
% data = importdata(filename, ','); % read the data
% x = data(:,1); % get the first column
% y = data(:,2); % get the second column
% plot(x,LineWidth=1) % plot the data
% hold on
% plot(y,LineWidth=1)
% 

xmin=0;
xmax=10000;
ymin = -1000;
ymax= 1000;
filename = 'teleop.txt';
figure;
subplot(2,1,1); 
h1 = plot(NaN,NaN,'LineWidth',1); %  empty plot for the first column
hold on;
h2 = plot(NaN,NaN,'LineWidth',1); % empty plot for the second column
legend('master','slave')
title('Teleoperation');
xlabel('Time (ms)');
ylabel('Position');

subplot(2,1,2); % 
h3 = plot(NaN,NaN,'LineWidth',1);
title('Error Plot');
xlabel('Time (ms)');
ylabel('Degrees Error');

% title('normalized error');
% xlabel('Time (ms)');
% ylabel('Error');

while true % infinite loop
    data = importdata(filename, ',');
    x = data(:,1); % first column
    y = data(:,2); % second column
    set(h1,'XData',1:numel(x),'YData',x); 
    set(h2,'XData',1:numel(y),'YData',y); 
    set(h3,'XData',1:numel(x),'YData',abs(x-y)); %  error
%     set(h3,'XData',1:numel(x),'YData',abs(x-y)./max(abs(x-y))); % normalized error
    drawnow; % force MATLAB update
   
end