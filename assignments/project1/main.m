%% DESCRIPTON
% Process laserdata receives in polar form from a laser scanner (LIDAR).
% This function reads the data, converts it into cartesian coordinates,
% extracts objects of interests (OOI) and shows the results in plots.
%
% This file is altered from the template file provide in the class.

function main(file)
% Add that folder plus all subfolders to the path.
dirs = fileparts(which(mfilename)); 
addpath(genpath(dirs));

% Params
start = 1;
increment = 1;
freq = 100; % [Hz]
t_pause = 1.0/freq; % [s]
settings.use_circle_fit = false;
settings.verbose = false;
    
global ABCD;
ABCD.flagPause=0;
% In case the caller does not specify the input argument, we propose a
% default one, assumed to be in the same folder where we run this example from.
if ~exist('file','var'), file ='Laser__2.mat'; end;

% Load Scans from file
load(file); 
% Create figure handle
h = create_figure(dataL, file);

% Loop through the scans.
N = dataL.N; 
i = start;
while 1
    
    if (ABCD.flagPause), pause(0.2) ; continue ; end
    if i>N, break ;  end
        
    % Native time expressed via uint32 numbers, where 1 unint means
    % [1/10,000]second (i.e. 0.1 millisecond)
    % (BTW: we do not use the time, in this task)
    t =  double(dataL.times(i)-dataL.times(1))/10000;
    % t: time expressed in seconds, relative to the time of the first scan.
    
    scan_i = dataL.Scans(:,i);
    ProcessingOfScan(scan_i, t, h, i, settings);   % Process scan

    pause(t_pause) ;                   % wait for ~10ms (approx.)
    i=i+increment;
end

fprintf('\nDONE!\n');

return;
end

function h = create_figure(dataL, file)
        % --------------------------------------
    % Create graphical object for refreshing data during program execution.
    figure(1) ; clf(); 
    
    h.all_scans = plot(0,0,'b.');      % all laser points
    hold on;
    h.HR = plot(0,0,'r+');             % highly reflective laser points
    h.OOI_brilliant = plot(0,0,'g*');  % objects of interest which are ...
                                       % brilliant
    
    axis([-10,10,0,20]);               % focuses plot on this region ( of interest in L220)
    xlabel('X [m]');
    ylabel('Y [m]');
    %     h.legend = legend('raw laserscan', 'highly reflecting', 'brilliant OOI');

    
    h.title = title('');           % create an empty title..

    zoom on ;  grid on;
            
    fprintf('\nThere are [ %d ] laser scans in this dataset (file [%s])\n',dataL.N,file);
    
    
    uicontrol('Style','pushbutton','String','Pause/Cont.','Position',[10,1,80,20],'Callback',{@FigureControlCallback,1});
    uicontrol('Style','pushbutton','String','END Now','Position',[90,1,80,20],'Callback',{@FigureControlCallback,2});

end

function ProcessingOfScan(scan, t, mh, i, settings)
    % Function to proccess data from scans and refreshs figure with new
    % data.
    % This function is altered from template file provided in class.
    %
    % INPUT:
    %       - 'scan' : scan measurements to be shown.
    %       - 't':  associated time.    
    %       - 'i' : scan number.
    %       - 'mh'  : struct contaning handles of necessary graphical objects.
    
    % Convert scan data to cartestian coordinates
    [X, Y, intensities] = convertScan2Cartesian(scan);
    
    % Extract OOI
    tic
    OOIs = ExtractOOIs(X, Y, intensities, settings.use_circle_fit);
    if settings.verbose
        toc
    end
    
    % refresh the data in the plot
    set(mh.all_scans,'xdata',X,'ydata',Y);
    set(mh.HR,'xdata',X(intensities ~= 0),'ydata',Y(intensities ~= 0));
    set(mh.OOI_brilliant,'xdata',OOIs.Centers(OOIs.Color ~= 0, 1), ...
                         'ydata',OOIs.Centers(OOIs.Color ~= 0, 2));
    
    % Set current title of figure
    s= sprintf('Laser scan # [%d] at time [%.3f] secs',i,t);
    set(mh.title,'string',s);
    
    return;
end

% Callback for figure control
function FigureControlCallback(~,~,x)   
    global ABCD;
        
    if (x==1)
       ABCD.flagPause = ~ABCD.flagPause; %Switch ON->OFF->ON -> and so on.
       return;
    end;
    if (x==2)
        
        disp('you pressed "END NOW"');
        uiwait(msgbox('Ooops, you still need to implement this one!','?','modal'));
        
        return;
    end;
    return;    
end