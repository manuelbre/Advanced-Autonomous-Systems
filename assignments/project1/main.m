%% DECRIPTON
% Process laserdata receives in polar form from a laser scanner (LIDAR).
% This function reads the data, converts it into cartesian coordinates,
% extracts objects of interests (OOI) and shows the results in plots.
%
% This file is altered from the template file provide in the class.

function main(file)

% Params
start = 1;
increment = 1; 
use_circle_fit = true;
    
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
    ProcessingOfScan(scan_i, t, h, i, use_circle_fit);   % Process scan

    pause(0.01) ;                   % wait for ~10ms (approx.)
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
    h.HR = plot(0,0,'r+');             % highly reflective aser points
    h.OOI_brilliant = plot(0,0,'g*');  % objects of interest which are ...
                                       % brilliant
    
    axis([-10,10,0,20]);                % focuses plot on this region ( of interest in L220)
    xlabel('Y [m]');
    ylabel('X [m]');
    
    h.title = title('');           % create an empty title..
    zoom on ;  grid on;
            
    fprintf('\nThere are [ %d ] laser scans in this dataset (file [%s])\n',dataL.N,file);
    
    
    uicontrol('Style','pushbutton','String','Pause/Cont.','Position',[10,1,80,20],'Callback',{@FigureControlCallback,1});
    uicontrol('Style','pushbutton','String','END Now','Position',[90,1,80,20],'Callback',{@FigureControlCallback,2});

end

function ProcessingOfScan(scan, t, mh, i, use_circle_fit)
    % Function to proccess data from scans and refreshs figure with new
    % data.
    % This function is altered from template file provided in class.
    %
    % INPUT:
    %       - 'scan' : scan measurements to be shown.
    %       - 't':  associated time.    
    %       - 'i' : scan number.
    %       - 'mh'  : struct contaning handles of necessary graphical objects.
    
    
    % scan data is provided as a array of class uint16, which encodes range
    % and intensity (that is the way the sensor provides the data, in that
    % mode of operation)
     
    % We extract range and intensity, here.
    %useful masks, for dealing with bits.
    mask1FFF = uint16(2^13-1);
    maskE000 = bitshift(uint16(7),13)  ;

    intensities = bitand(scan,maskE000);  
    ranges    = single(bitand(scan,mask1FFF))*0.01; % [m]
    % Ranges expressed in meters, and unsing floating point format (e.g. single).

    % 2D points, expressed in Cartesian. From the sensor's perpective.
    angles = [0:360]'*0.5* pi/180 ;  % associated angle, for each individual range in a scan
    X = cos(angles).*ranges; % [m]
    Y = sin(angles).*ranges; % [m]
    
    % Extract OOI
    tic
    OOIs = ExtractOOIs(X, Y, intensities, use_circle_fit);
    toc
    
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