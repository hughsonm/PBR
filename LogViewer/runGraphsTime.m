function [] = runGraphsTime(data,names,grouping)

% grouping tells us which indices go on which subplots:
% The indices in column 1 go on subplot 1.
% The indices in column 2 go on subplot 2.
% ...

    FigStruct = InitializeGraphs(data,names,grouping);
    set(FigStruct.ParentFigure, 'WindowButtonDownFcn', {@startDragFcn,FigStruct});
    set(FigStruct.ParentFigure, 'WindowButtonUpFcn', {@stopDragFcn,FigStruct});
    set(FigStruct.ParentFigure, 'keypressfcn', {@keyPressFcn,FigStruct});
end

function keyPressFcn(...
	src,code,...
	figstruct)
	ch_pressed = code.Character;
	zoomadd = 0;
	switch ch_pressed
        case '='
            zoomadd = +0.1;
        case '-'
            zoomadd = -0.1;
        otherwise
            zoomadd = 0.0;
	end
	zoomfactor = 1+zoomadd;
    if zoomfactor <= 0
        zoomfactor = 0.1;    
    end
	cp = get(figstruct.ParentFigure,'currentpoint');
	if(cp(1)<0.7)
		% get the axes' x- and y-limits
		xlim = get(gca, 'xlim');
		% get the current camera position, and save the [z]-value
		cam_pos_Z = get(gca, 'cameraposition');
		cam_pos_Z = cam_pos_Z(3);
		% get the current point
		old_position = get(gca, 'CurrentPoint');
		old_position(1,3) = cam_pos_Z;
		% calculate zoom factor		
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
    for ii = 1:length(figstruct.listboxes)
		set(figstruct.listboxes(ii),'callback',{@listBoxSelFcn,figstruct})
	end
	set(figstruct.ParentFigure, 'WindowButtonDownFcn', {@startDragFcn,figstruct});
	set(figstruct.ParentFigure, 'WindowButtonUpFcn', {@stopDragFcn,figstruct});
	set(figstruct.ParentFigure, 'keypressfcn', {@keyPressFcn,figstruct});
	linkaxes(figstruct.subgraphs, 'x');
end

function startDragFcn(...
  src,code,...
  figstruct)
  % Make the horizontal bar track where the mouse pointer goes
  set(figstruct.ParentFigure, 'WindowButtonMotionFcn',{@draggingFcn,figstruct});
end

function draggingFcn(...
	src,code,...
	figstruct)
	pt = get(figstruct.subgraphs(1), 'CurrentPoint');
	set(figstruct.timebox,...
	'string', strcat('Time: ', num2str(round(pt(1)*100)/100)));
	tt = figstruct.data(:,1)/1e7;
	closestTime = abs(tt - pt(1));
	[timeInt, timeRow] = min(closestTime);
	idcs_to_disp = figstruct.grouping(figstruct.grouping~=0);
	idcs_to_disp = idcs_to_disp(:)';
	didx = 1;
	for idx = idcs_to_disp
		label = strcat(figstruct.names{idx},': ',num2str(figstruct.data(timeRow,idx)));
		set(figstruct.infoboxes(didx), 'String', label);
		didx = didx+1;
	end
	for idx = 1:length(figstruct.subgraphs)
		set(figstruct.vertmarkers(idx),...
			'XData', pt(1)*[1 1]);
		full_mod_data_row = figstruct.mod_data(timeRow,:);
		part_idcs = figstruct.grouping(:,idx).';
		part_idcs = part_idcs(part_idcs ~=0);
		mod_row_part = full_mod_data_row(part_idcs);
		h_bot = min(0,min(mod_row_part));
		h_top = max(0,max(mod_row_part));
		set(figstruct.vertmarkers(idx),...
			'YData', [h_bot,h_top]);
	end
	for ii = 1:length(figstruct.listboxes)
		set(figstruct.listboxes(ii),'callback',{@listBoxSelFcn,figstruct})
	end
	set(figstruct.ParentFigure, 'WindowButtonDownFcn', {@startDragFcn,figstruct});
	set(figstruct.ParentFigure, 'WindowButtonUpFcn', {@stopDragFcn,figstruct});
	set(figstruct.ParentFigure, 'keypressfcn', {@keyPressFcn,figstruct});
    linkaxes(figstruct.subgraphs, 'x');
end
function stopDragFcn(...
    src,code,...
    figstruct)
    % Once the mouse button gets released, stop following the pointer.
    set(figstruct.ParentFigure, 'WindowButtonMotionFcn', '')
end


function listBoxSelFcn(...
	src,code,...
	figstruct)
	ibox = str2num(get(src,'tag'));
	idcs = get(figstruct.listboxes(ibox),'value');
	idcs = idcs(:);
	grouping = figstruct.grouping;
	if(length(idcs)>size(grouping,1))
		grouping = [grouping;zeros(length(idcs)-size(grouping,1),size(grouping,2))];
	end
	grouping(:,ibox)=0;
	grouping(1:length(idcs),ibox) = idcs;
	subplot(figstruct.subgraphs(ibox));
	modSet = figstruct.mod_data(:,idcs);
	plot(figstruct.data(:,1)/1e7, modSet);
	legend(figstruct.names(idcs));
	grid on;
	grid minor;
	ax = gca;
	set(ax,'GridAlpha',0.4);
	set(ax,'MinorGridAlpha',0.5);
	figstruct.vertmarkers(ibox) = line(...
		[1,1],...
		[0,max(max(modSet))],...
		'color',[0,.75,0],...
		'LineWidth',2);

	[plotsPer,nPlots] = size(grouping);
	nBoxes = sum(sum(grouping ~= 0));
	boxPos = (linspace(.1,.9,nBoxes));
	boxPos = boxPos(end:-1:1);
	for jj = length(figstruct.infoboxes):-1:1
		delete(figstruct.infoboxes(jj))
	end
	uiOffset = 0;
	figstruct.infoboxes = [];
	for idx = grouping(grouping~=0).'
		uiOffset = uiOffset+1;
		figstruct.infoboxes(uiOffset) = uicontrol(figstruct.ParentFigure,...
			'Style', 'text',...
			'Units','normalized',...
			'Position', [.1 boxPos(uiOffset) .1 .05]);
  	end
	figstruct.grouping = grouping;
	% We have modified FigStruct by changing what gets displayed. This means we
	% should redefine all the callbacks, too.
	for ii = 1:length(figstruct.listboxes)
		set(figstruct.listboxes(ii),'callback',{@listBoxSelFcn,figstruct})
	end
	set(figstruct.ParentFigure, 'WindowButtonDownFcn', {@startDragFcn,figstruct});
	set(figstruct.ParentFigure, 'WindowButtonUpFcn', {@stopDragFcn,figstruct});
	set(figstruct.ParentFigure, 'keypressfcn', {@keyPressFcn,figstruct});
    linkaxes(figstruct.subgraphs, 'x');
end

function FigStruct = InitializeGraphs(...
        data,...
        names,...
        grouping)
    FigStruct.data = data;
    FigStruct.names = names;
    FigStruct.grouping = grouping;
    FigStruct.ParentFigure = figure(...
        'units','normalized',...
        'outerposition',[0 0 1 1]);
    set(FigStruct.ParentFigure, 'menubar', 'none');
    
    [plotsPer,nPlots] = size(grouping);
    nBoxes = sum(sum(grouping ~= 0));
    boxPos = (linspace(.1,.9,nBoxes));
    boxPos = boxPos(end:-1:1);
    
    pHeight = (1/nPlots)*.9;
    vertFillRatio = 0.9;
    
    TIME_SCALE = 1E7;
    tt = data(:,1)/TIME_SCALE;
    ii = 0;
    
    uiOffset = 0;
    FigStruct.mod_data = (data./(10.^floor(log10(max(data)))));
    
    for gg = grouping
        ii = ii + 1;
        FigStruct.subgraphs(ii) = subplot(...
            'Position',...
            [.02, .98-ii*pHeight, .7, pHeight*vertFillRatio]);
        idcs = gg';
        idcs = idcs(idcs ~= 0);
        modSet = FigStruct.mod_data(:,idcs);
        plot(tt, modSet);
        legend(names(idcs));
        grid on;
        grid minor;
        ax = gca;
        set(ax,"GridAlpha", 0.4);
        set(ax,"MinorGridAlpha",0.5);
        FigStruct.vertmarkers(ii) = line(...
            [1,1],[0,max(max(modSet))],...
            'color',[0,.75,0],...
            'LineWidth',2);
        uic = uicontrol(FigStruct.ParentFigure,...
            'Style','listbox',...
            'Units', 'normalized',...
            'Position',[0.75,0.98-ii*pHeight,0.2,pHeight],...
            'string',names,...
            'Max',10,...
            'Min',0,...
            'Value',idcs,...            
            'Tag',num2str(ii));
        FigStruct.listboxes(ii) = uic;
        
        for idx = idcs
            uiOffset = uiOffset+1;
            FigStruct.infoboxes(uiOffset) = uicontrol(FigStruct.ParentFigure,...
                'Style', 'text',...
                'Units','normalized',...
                'Position', [.1 boxPos(uiOffset) .1 .05]);
        end
        
    end
	FigStruct.timebox = uicontrol(FigStruct.ParentFigure,...
		'Style', 'text',...
		'Units', 'normalized',...
		'Position', [.4 .00 .1 .02]);
	for ii = 1:length(FigStruct.listboxes)
		set(FigStruct.listboxes(ii),'callback',{@listBoxSelFcn,FigStruct})
	end
	set(FigStruct.ParentFigure, 'WindowButtonDownFcn', {@startDragFcn,FigStruct});
	set(FigStruct.ParentFigure, 'WindowButtonUpFcn', {@stopDragFcn,FigStruct});
	set(FigStruct.ParentFigure, 'keypressfcn', {@keyPressFcn,FigStruct});
    linkaxes(FigStruct.subgraphs, 'x');
end



