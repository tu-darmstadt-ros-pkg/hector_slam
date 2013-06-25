function output = rotationGUI(I, n_lines)
    %# setup GUI
    figure(1);
    hFig = gcf;
    set(hFig,'menu','none');
    hAx = axes('Parent',hFig);
    uicontrol('Parent',hFig, 'Style','slider', 'Value',0, 'Min',-10,...
        'Max',10, 'SliderStep',[.01 .001], ...
        'Position',[150 5 300 20], 'Callback',@slider_callback) 
    hTxt = uicontrol('Style','text', 'Position',[290 28 40 15], 'String','0.000');
    hBut = uicontrol('Position',[480 5 80 20],'String','Done',...
              'Callback','uiresume(gcbf)');
    
    function grid_image(image, n_lines)
        imagesc(image, 'Parent',hAx)
        axis image
        colormap gray

        xlim = get(gca,'XLim');
        ylim = get(gca,'YLim');

        hold on
        for x = xlim(1):diff(xlim)/n_lines:xlim(2)
            plot([x x], ylim, 'b-');
        end
        for y = ylim(1):diff(ylim)/n_lines:ylim(2)
            plot(xlim, [y y], 'b-');
        end
        hold off; 
    end

    grid_image(I, n_lines);
    
    angle = 0.0;

    %# Callback function
    function slider_callback(hObj, eventdata)
        angle = get(hObj,'Value');
        grid_image(imrotate(I,angle), n_lines);
        set(hTxt, 'String',num2str(angle))       %# update text
    end

    uiwait
    disp(['Final Angle: ' num2str(angle)]);
    output = imrotate(I,angle,'bilinear','crop');
end