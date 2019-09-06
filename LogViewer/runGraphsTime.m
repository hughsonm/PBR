function [] = runGraphsTime(data,names,grouping)

% grouping tells us which indices go on which subplots:
% The indices in column 1 go on subplot 1.
% The indices in column 2 go on subplot 2.
% ...

[timeGraphs,graphss,data,mod_data,hh,lb,dispdata] = InitializeGraphs(data,names,grouping);

linkaxes(graphss, 'x');

set(timeGraphs, 'WindowButtonDownFcn', @startDragFcn);
set(timeGraphs, 'WindowButtonUpFcn', @stopDragFcn);
set(timeGraphs, 'WindowScrollWheelFcn', @doScroll);
timeBox = uicontrol(timeGraphs, 'Style', 'text',...
    'Units', 'normalized',...
    'Position', [.4 .00 .1 .02]);

    function startDragFcn(varargin)
        % Make the horizontal bar track where the mouse pointer goes
        set(timeGraphs, 'WindowButtonMotionFcn', @draggingFcn)
    end
    function draggingFcn(varargin)
        pt = get(graphss(1), 'CurrentPoint');
        set(timeBox, 'string', strcat('Time: ', num2str(round(pt(1),2))));
        tt = data(:,1)/1e7;
        closestTime = abs(tt - pt(1));
        [timeInt, timeRow] = min(closestTime);
        idcs_to_disp = grouping(grouping~=0);
        idcs_to_disp = idcs_to_disp(:);
        didx = 1;
        for idx = idcs_to_disp
            label = strcat(names{idx},': ',num2str(data(timeRow,idx)));
            set(dispdata(didx), 'String', label);
            didx = didx+1;
        end
        for idx = 1:length(graphss)
            set(hh(idx), 'XData', pt(1)*[1 1]);
            full_mod_data_row = mod_data(timeRow,:);
            part_idcs = grouping(:,idx).';
            part_idcs = part_idcs(part_idcs ~=0);
            mod_row_part = full_mod_data_row(part_idcs);
            h_bot = min(0,min(mod_row_part));
            h_top = max(0,max(mod_row_part));
            set(hh(idx), 'YData', [h_bot,h_top]);
        end
    end

    function stopDragFcn(varargin)
        % Once the mouse button gets released, stop following the pointer.
        set(timeGraphs, 'WindowButtonMotionFcn', '')
    end

    function doScroll(varargin)
        % get the amount of scolls
        if(timeGraphs.CurrentPoint(1)<0.7)
            scrolls = varargin{2}.VerticalScrollCount;
            % get the axes' x- and y-limits
            xlim = get(gca, 'xlim');
            % get the current camera position, and save the [z]-value
            cam_pos_Z = get(gca, 'cameraposition');
            cam_pos_Z = cam_pos_Z(3);
            % get the current point
            old_position = get(gca, 'CurrentPoint');
            old_position(1,3) = cam_pos_Z;
            % calculate zoom factor
            zoomfactor = max(0.01,1 - scrolls/30);
            % adjust camera position
            set(gca, 'cameratarget', [old_position(1, 1:2), 0],...
                'cameraposition', old_position(1, 1:3));
            % adjust the camera view angle (equal to zooming in)
            camzoom(zoomfactor);
            
            % zooming with the camera has the side-effect of
            % NOT adjusting the axes limits. We have to correct for this:
            x_lim1 = (old_position(1,1) - min(xlim))/zoomfactor;
            x_lim2 = (max(xlim) - old_position(1,1))/zoomfactor;
            xlim   = [old_position(1,1)-x_lim1,old_position(1,1)+x_lim2];
            set(gca, 'xlim', xlim) 
            
            % set new camera position
            new_position = get(gca, 'CurrentPoint');
            old_camera_target =  get(gca, 'CameraTarget');
            old_camera_target(3) = cam_pos_Z;
            new_camera_position = old_camera_target - ...
                (new_position(1,1:3) - old_camera_target(1,1:3));
            % adjust camera target and position
            set(gca, 'cameraposition', new_camera_position(1, 1:3),...
                'cameratarget', [new_camera_position(1, 1:2), 0]);
            % we also have to re-set the axes to stretch-to-fill mode
            set(gca, 'cameraviewanglemode', 'auto',...
                'camerapositionmode', 'auto',...
                'cameratargetmode', 'auto');
        end
        %disp(timeGraphs.CurrentPoint);
    end % scroll_zoom




    function listBoxSelFcn(varargin)
        ibox = str2num(varargin{1}.Tag);
        idcs = lb(ibox).Value.';
        if(length(idcs)>size(grouping,1))
            grouping = [grouping;zeros(length(idcs)-size(grouping,1),size(grouping,2))];
        end
        grouping(:,ibox)=0;
        grouping(1:length(idcs),ibox) = idcs;
        subplot(graphss(ibox));
        modSet = mod_data(:,idcs);
        plot(data(:,1)/1e7, modSet);
        legend(names(idcs));
        grid on;
        grid minor;
        ax = gca;
        ax.GridAlpha = 0.4;
        ax.MinorGridAlpha = 0.5;
        hh(ibox) = line([1,1],[0,max(max(modSet))],'color',[0,.75,0],'LineWidth',2);
        
        [plotsPer,nPlots] = size(grouping);
        nBoxes = sum(sum(grouping ~= 0));
        boxPos = (linspace(.1,.9,nBoxes));
        boxPos = boxPos(end:-1:1);
        for jj = length(dispdata):-1:1
            delete(dispdata(jj))
        end
        uiOffset = 0;
        for idx = grouping(grouping~=0).'
            uiOffset = uiOffset+1;
            dispdata(uiOffset) = uicontrol(timeGraphs,...
                'Style', 'text',...
                'Units','normalized',...
                'Position', [.1 boxPos(uiOffset) .1 .05]);
        end                
    end

    function [timeGraphs,graphss,data,mod_data,hh,lb,disp_data] = InitializeGraphs(...
            data,...
            names,...
            grouping)
        timeGraphs = figure(...
            'units','normalized',...
            'outerposition',[0 0 1 1]);
        set(timeGraphs, 'menubar', 'none');
        
        [plotsPer,nPlots] = size(grouping);
        nBoxes = sum(sum(grouping ~= 0));
        boxPos = (linspace(.1,.9,nBoxes));
        boxPos = boxPos(end:-1:1);
        
        pHeight = (1/nPlots)*.9;
        
        TIME_SCALE = 1E7;
        tt = data(:,1)/TIME_SCALE;
        ii = 0;
        
        uiOffset = 0;
        mod_data = (data./(10.^floor(log10(max(data)))));
        
        for gg = grouping
            ii = ii + 1;
            graphss(ii) = subplot(...
                'Position',...
                [.02, .98-ii*pHeight, .7, pHeight]);
            idcs = gg';
            idcs = idcs(idcs ~= 0);
            modSet = mod_data(:,idcs);
            plot(tt, modSet);
            legend(names(idcs));
            grid on;
            grid minor;
            ax = gca;
            set(ax,"GridAlpha", 0.4);
            set(ax,"MinorGridAlpha",0.5);
            hh(ii) = line(...
                [1,1],[0,max(max(modSet))],...
                'color',[0,.75,0],...
                'LineWidth',2);
            uic = uicontrol(timeGraphs,...
                'Style','listbox',...
                'Units', 'normalized',...
                'Position',[0.75,0.98-ii*pHeight,0.2,pHeight],...
                'string',names,...
                'Max',10,...
                'Min',0,...
                'Value',idcs,...
                'CallBack',@listBoxSelFcn,...
                'Tag',num2str(ii));
            lb(ii) = uic;
            
            for idx = idcs
                uiOffset = uiOffset+1;
                disp_data(uiOffset) = uicontrol(timeGraphs,...
                    'Style', 'text',...
                    'Units','normalized',...
                    'Position', [.1 boxPos(uiOffset) .1 .05]);
            end
            
        end
    end
end



