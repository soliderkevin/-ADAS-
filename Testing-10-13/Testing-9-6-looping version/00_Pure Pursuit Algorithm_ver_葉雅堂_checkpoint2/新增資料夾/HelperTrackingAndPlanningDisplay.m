classdef HelperTrackingAndPlanningDisplay < matlab.System
    properties
        BirdsEyeView
        ChaseView
    end

    properties
        Figure
        Record = false;
        FigureSnapshots
        pFrames = cell(0,1);
        SnapTimes = [17.5 20 20.5 30];
    end


    methods
        function snapnow(obj)
            if isPublishing(obj)
                panels = findall(obj.Figure,'Type','uipanel');
                f = copy(panels);
                obj.FigureSnapshots{end+1} = f;
            end
        end
        
        function fOut = showSnaps(obj, panelIdx, idx)
            fOut = [];
            if isPublishing(obj)
                if nargin == 1
                    idx = 1:numel(obj.FigureSnapshots);
                end

                fOut = gobjects(numel(idx),1);

                for i = 1:numel(idx)
                    f = figure('Units','normalized','Position',[0 0 0.5 0.5]);
                    copy(obj.FigureSnapshots{idx(i)}(panelIdx),f);
                    panel = findall(f,'Type','uipanel');
                    panel.Position = [0 0 1 1];
                    ax = findall(panel,'Type','Axes');
                    for k = 1:numel(ax)
                        ax(k).XTickLabelMode = 'auto';
                        ax(k).YTickLabelMode = 'auto';
                    end
                    uistack(panel,'top');
                    fOut(i) = f;
                end
            end
        end
        
        function writeAnimation(obj, fName, delay)
            if nargin == 2
                delay = 0;
            end
            downsampleFactor = 2;
            if obj.Record
                frames = obj.pFrames;
                imSize = size(frames{1}.cdata);
                im = zeros(imSize(1),imSize(2),1,floor(numel(frames)/downsampleFactor),'uint8');
                map = [];
                count = 1;
                for i = 1:downsampleFactor:numel(frames)
                    if isempty(map)
                        [im(:,:,1,count),map] = rgb2ind(frames{i}.cdata,256,'nodither');
                    else
                        im(:,:,1,count) = rgb2ind(frames{i}.cdata,map,'nodither');
                    end
                    count = count + 1;
                end
                imwrite(im,map,[fName,'.gif'],'DelayTime',delay,'LoopCount',inf);
            end
        end
    end

    methods (Access = protected)
        function setupImpl(obj,scenario, varargin)
            f = figure('Units','normalized','Position',[0.1 0.1 0.75 0.7]);
            obj.Figure = f;
            p1 = uipanel(f,'Units','normalized','Position',[0 0 0.7 1]);
            ax = axes(p1);
            obj.ChaseView = HelperTrackingAndPlanningChasePlot('Parent',ax);
            p2 = uipanel(f,'Units','normalized','Position',[0.7 0 0.3 1]);
            ax2 = axes(p2);
            obj.BirdsEyeView = HelperTrackingAndPlanningBirdsEyePlot('Parent',ax2);
            setup(obj.BirdsEyeView,scenario,varargin{:});
            setup(obj.ChaseView,scenario,varargin{:});
            deleteLegend(obj.BirdsEyeView);
            obj.BirdsEyeView.Parent.XAxis.Visible = 'off';
            obj.BirdsEyeView.Parent.YAxis.Visible = 'off';
            obj.BirdsEyeView.Parent.Position = [0 0 1 1];
            if isPublishing(obj)
                obj.Figure.Visible = 'off';
            end
        end

        function stepImpl(obj,scenario, egoVehicle, varargin)
            obj.BirdsEyeView(scenario, egoVehicle, varargin{:});
            obj.ChaseView(scenario, egoVehicle, varargin{:});
            if obj.Record
                obj.pFrames{end+1} = getframe(obj.Figure);
            end
            if any(abs(scenario.SimulationTime - obj.SnapTimes) < 0.05)
                snapnow(obj);
            end
        end
        
        function tf = isPublishing(~)
            tf = false;
            try
                s = numel(dbstack);
                tf = s > 5;
            catch
            end
        end
    end
end
