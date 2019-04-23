function h = create_figure()
    % --------------------------------------
    % Create graphical object for refreshing data during program execution.
    figure() ; clf(); 
    
    % Parmas
    n_txt = 10;
    
    hold on;
    h.OOI_brilliant_init = plot(0, 0, 'g*', 'MarkerSize',10);  % Initial ...
                                            % brilliant objects of interest
    
    h.OOI_brilliant = plot(0, 0, 'r+', 'MarkerSize', 10); % Brilliant ....
                                            % object of interest.
    h.OOI_brilliant_legacy = plot(0, 0, 'b+', 'MarkerSize', 10); % Brilliant ....
                                            % object of interest.
    
    % Text
    h.OOI_brilliant_init_txt = cell(n_txt,1);
    h.OOI_brilliant_txt = cell(n_txt,1);
    for i = 1:n_txt
            h.OOI_brilliant_init_txt{i} = text(0,0, '', 'Color', 'g');
            h.OOI_brilliant_txt{i} = text(0,0, '', 'Color', 'r');
    end
    
    % Speed and Bias text
    h.v_estimated = text(3, 8, '', 'Color', 'k');
    h.v = text(3, 7, '', 'Color', 'k');
    h.bias_estimated = text(3, 6, '', 'Color', 'k');

    
    % Robot Position with arrow w/ EKF
    h.robot = quiver([],[],[],[],0);
    h.robot.Color = 'm';
    h.robot.LineWidth = 2;
    h.robot.MaxHeadSize = 1;
    
    % Robot Position with arrow w/o EKF
    h.robot_legacy = quiver([],[],[],[],0);
    h.robot_legacy.Color = 'b';
    h.robot_legacy.LineWidth = 2;
    h.robot_legacy.MaxHeadSize = 1;

    % focuses plot on this region of interest
    axis([-10,10,-10,10]);               
    xlabel('X [m]');
    ylabel('Y [m]');
    h.legend = legend('OOIs Map','OOIs w/ EKF', 'OOIs w/o EKF', ...
        'Robot w/ EKF', 'Robot w/o EKF', 'Location', 'East', 'AutoUpdate','off');
    
    % Title
    h.title = title('');

    zoom on ;  grid on;    
    
    uicontrol('Style','pushbutton','String','Pause/Cont.','Position',[10,1,80,20],'Callback',{@FigureControlCallback,1});
    uicontrol('Style','pushbutton','String','END Now','Position',[90,1,80,20],'Callback',{@FigureControlCallback,2});

end

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