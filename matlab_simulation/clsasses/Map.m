classdef Map < handle
    properties
        obs_list = {};
        map_figure = figure("Name",'map','Tag','map_fig')
        map_span = [-8 8; -8 8];
    end
    methods (Access = private)
        function obj = Map()
            % Private constructor for the singleton class
        end
    end
    methods (Static)
        function singleObj = getInstance()
            persistent uniqueInstance
            if isempty(uniqueInstance)
                uniqueInstance = Map();
            end
            singleObj = uniqueInstance;
        end
    end
    methods
        function add_polygon(obj,geo)
            obj.obs_list{end + 1} = Obstacle('polygon',geo);
        end
        function draw_all_obs(obj)
            n = numel(obj.obs_list);
            figure(obj.map_figure);hold on;
            xlim(obj.map_span(1,:));ylim(obj.map_span(2,:))
            for i = 1:n
                obj.obs_list{i}.draw_on_map()
            end
        end
    end
end