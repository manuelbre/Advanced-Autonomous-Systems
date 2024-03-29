function OOIs = plotLaserdataGlobalframe(scan, t, X, plot_handle_laser, ...
                    plot_handle_robot, titel_handle, idx, settings)
    %% Description
    % Function to proccess data from scans and refreshs figure with new
    % data. The OOI are plotted in the global coordinate frame.
    %
    % INPUT:
    %       - 'scan' : Scan data
    %       - 't':  Current time
    %       - 'X' (3X1) : Current state of robot in global coordinate system 
    %       - 'plot_handle' : Handle to plot
    %       - 'titel_handle' : Handle to plot title
    %       - 'idx' : Scan number
    %       - 'settings'  : Settings for this function
    
    %% Function
    
    % Convert scan data to cartestian coordinates
    [x_L, y_L, intens] = convertScan2Cartesian(scan);
    
    tic
    % Extract OOI
    OOIs = ExtractOOIs(x_L, y_L, intens, settings.use_circle_fit);
    
    % Transform to Global coordinate frame
    OOIs.Centers_G = zeros(size(OOIs.Centers));
    [OOIs.Centers_G(:,1), OOIs.Centers_G(:,2)] = Laser2Global(OOIs.Centers(:,1), OOIs.Centers(:,2), X, settings.d);    
    if settings.verbose
        toc
    end
    
    % Refresh the data in the plot
    % Get brilliant OOI
    x_G = OOIs.Centers_G(OOIs.Color ~= 0, 1);
    y_G = OOIs.Centers_G(OOIs.Color ~= 0, 2);
    % OOI
    set(plot_handle_laser,'xdata', x_G, 'ydata', y_G);
    % Robot Location
    set(plot_handle_robot,'XData', X(1), 'YData', X(2), 'UData',...
                    cos(X(3)) ,'VData', sin(X(3)));
    
    % Set current title of figure
    s = sprintf('Laser scan # [%d] at time [%.3f] secs', idx, t);
    set(titel_handle,'string',s);
    
end
