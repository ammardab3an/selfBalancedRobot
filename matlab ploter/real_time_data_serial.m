fclose(s);

SerialPort='com7'; %serial port
MaxDeviation = 10000;%Maximum Allowable Change from one value to next 
TimeInterval = 0;%time interval between each input.
loop = 120;%count values
%%Set up the serial port object

s = serial(SerialPort, 'BaudRate', 19200);
fopen(s);

time = now;
voltage = 0;

%% Set up the figure 
figureHandle = figure('NumberTitle','off',...
    'Name','Angle Characteristics',...
    'Color',[0 0 0],'Visible','off');

% Set axes
axesHandle = axes('Parent',figureHandle,...
    'YGrid','on',...
    'YColor',[0.9725 0.9725 0.9725],...
    'XGrid','on',...
    'XColor',[0.9725 0.9725 0.9725],...
    'Color',[0 0 0]);

hold on;

plotHandle = plot(axesHandle,time,voltage,'Marker','.','LineWidth',1,'Color',[0 1 0]);

xlim(axesHandle,[min(time) max(time+0.001)]);
ylim(axesHandle,[-50, +50]);

% Create xlabel
xlabel('Time','FontWeight','bold','FontSize',14,'Color',[1 1 0]);

% Create ylabel
ylabel('Angle in Deg','FontWeight','bold','FontSize',14,'Color',[1 1 0]);

% Create title
title('Real Time Data','FontSize',15,'Color',[1 1 0]);



%% Initializing variables

voltage(1)=0;
time(1)=0;
count = 2;
k=1;

while 1
    
    %%Serial data accessing 
    
    input_raw = fscanf(s);
    input_raw
    input = sscanf(input_raw,'%f/%f/%f/%f/%f/%f/%f/%f/%f/%f/%f/%f/%f/%f/');
    
    sz = size(input);
    
    if sz(1) == 14
        voltage(count) = input(2);
        time(count) = count;
        set(plotHandle,'YData',voltage,'XData',time);
        set(figureHandle,'Visible','on');
        datetick('x','mm/DD HH:MM');
        pause(TimeInterval);
    end
    
    count = count +1;
    
    if count == 100
        count = 1;
    end
end



%% Clean up the serial port
fclose(s);
