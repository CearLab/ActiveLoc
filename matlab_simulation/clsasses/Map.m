classdef Map < handle
    properties
        obs_list = {};
        % map_figure = figure("Name",'map','Tag','map_fig')
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
        function set_figure()
            f = findobj('tag','map_fig');
            if isempty(f)
                f = figure("Name",'map','Tag','map_fig');
            end
            figure(f);hold on;box on; grid on;
        end
    end
    methods
        function add_polygon(obj,geo)
            obj.obs_list{end + 1} = Obstacle('polygon',geo);
        end

        function draw_all_obs(obj)
            n = numel(obj.obs_list);
            Map.set_figure();
            xlim(obj.map_span(1,:));ylim(obj.map_span(2,:))
            for i = 1:n
                obj.obs_list{i}.draw_on_map()
            end
        end
        function draw_all_teams(obj)
            manager = AgentManager.getInstance();
            team_list = manager.get_all_teams();
            n = numel(team_list);
            Map.set_figure();
            xlim(obj.map_span(1,:));ylim(obj.map_span(2,:))
            for i = 1:n
               team_list{i}.plot_team();
            end
        end
        function los_table = calc_los_map(obj)
            manager = AgentManager.getInstance();
            agents = manager.get_all_agent();
            na = numel(agents);
            los_table = [];
            for i = 1:(na - 1)
                ma = i + 1;
                i_loc = agents{i}.location;
                i_num = agents{i}.agent_number;
                for j = ma:na
                    j_loc = agents{j}.location;
                    j_num = agents{j}.agent_number;
                    los_table(end+1,:) = [i_num,j_num,i_loc,j_loc];
                end
            end
        end
        function draw_los_map(obj)
            Map.set_figure();
            los_table = obj.calc_los_map();
            n = size(los_table);
            n = n(1);
            for i = 1:n
                c_line = los_table(i,:);
                x1 = c_line(3);
                x2 = c_line(5);
                y1 = c_line(4);
                y2 = c_line(6);
                plot([x1,x2],[y1,y2],'Color','k');
            end
         

        end
    end
end