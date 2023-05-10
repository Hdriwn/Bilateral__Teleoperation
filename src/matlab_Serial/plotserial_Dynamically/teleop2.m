xmin=0;
xmax=10000;
ymin = -100;
ymax= 100;

txmin=0;
txmax=10000;
tymin = -8;
tymax= 8;

filename = 'teleop.txt'; 
figure; 
subplot(2,1,1);
h1 = plot(NaN,NaN,'LineWidth',1); 
hold on;
h2 = plot(NaN,NaN,'LineWidth',1); % empty plot for the second column
legend('master','slave')
title('Teleoperation');
xlabel('Time (ms)');
ylabel('Position');

subplot(2,1,2);
h3 = plot(NaN,NaN,'LineWidth',1); %empty plot for the error
title('Error');
xlabel('Time (ms)');
ylabel('Error');

while true 
    data = importdata(filename, ','); 
%     valid_rows = ~any(isnan(data), 2); 
%     data = data(valid_rows,:); 
    x = data(:,1);
    y = data(:,2); 
    set(h1,'XData',1:numel(x),'YData',x); 
    set(h2,'XData',1:numel(y),'YData',y);
    set(h3,'XData',1:numel(x),'YData',abs(x-y)); 

    xmin = max(1,numel(x)-200); % set xmin to be 100 time steps before the current time step
    xmax = numel(x); % set xmax to be the current time step
    txmin = max(1,numel(x)-200); % set xmin to be 100 time steps before the current time step
    txmax = numel(x); % set xmax to be the current time step
    subplot(2,1,1);
     xlim([xmin xmax]);
    ylim([ymin ymax]);
    subplot(2,1,2);
    xlim([txmin txmax]);
   ylim([tymin tymax]);
    
    drawnow; % force MATLAB to update the figure
   
end