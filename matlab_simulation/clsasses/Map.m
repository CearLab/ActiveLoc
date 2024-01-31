%% Map class
% file: Map.m
% author: Ido Sherf 
% date: 22/01/2024
% description: handle class handling the implementation and population of a
% 2D map of an environment. Specifically, this class is a coded as 
% singleton, namely it is ensured that the class has only one instance and 
% it is provided a way to access that instance from any point in the 
% application.
classdef Map < handle

    % properties of the cass
    properties

        % list of objects: cell array 
        obs_list = {};

        % dev
        % map_figure = figure("Name",'map','Tag','map_fig')

        % region covered by the map [m]
        map_span = [-8 8; -8 8];
    end

    % methods of the class (private)
    methods (Access = private)

        % Private constructor for the singleton class
        function obj = Map()
            
        end
    end

    % methods of the class (static)
    methods (Static)

        % get a single class instance from everywhere in the application
        function singleObj = getInstance()

            % define persistent variable (still exists after the call)
            persistent uniqueInstance

            % if it is empty, call the constructor
            if isempty(uniqueInstance)
                uniqueInstance = Map();
            end

            % assign the access point to the persistent var
            singleObj = uniqueInstance;
        end

        % graphic setup of the map plot
        function set_figure()

            % find if there exist a figure (specific tag,name)
            f = findobj('tag','map_fig');

            % if not, create it 
            if isempty(f)
                f = figure("Name",'map','Tag','map_fig');
            end

            % set base graphics
            figure(f);hold on;box on; grid on;
        end
    end

    % methodsof the class
    methods

        % add a polygon to the environment
        function add_polygon(obj,geo)

            % call the obstacle method (see Obstacle class)
            obj.obs_list{end + 1} = Obstacle('polygon',geo);

        end

        % plot on figure all obstacles
        function draw_all_obs(obj)

            % get number of ostacles
            n = numel(obj.obs_list);

            % call static method
            Map.set_figure();

            % set environment limits from class property
            xlim(obj.map_span(1,:));ylim(obj.map_span(2,:))

            % draw the obstacles (see Obstacle class)
            for i = 1:n
                obj.obs_list{i}.draw_on_map()
            end

        end

        % plot all teams of heterogeneous objects
        function draw_all_teams(obj)

            % get the Agent managers
            manager = AgentManager.getInstance();

            % get list of teams
            team_list = manager.get_all_teams();

            % get number of teams
            n = numel(team_list);

            % call static method
            Map.set_figure();

            % set environment limits from class property
            xlim(obj.map_span(1,:));ylim(obj.map_span(2,:))

            % draw the team (see team class)
            for i = 1:n
               team_list{i}.plot_team();
            end

        end

        % get all the LOS pairs between teams 
        % HERE WE SHOULD ADD SENSING RANGE
        function los_table = calc_los_map(obj)

            % get the Agent managers
            manager = AgentManager.getInstance();

            % get all the agents
            agents = manager.get_all_agent();

            % get number of agents
            na = numel(agents);

            % init emptu list
            los_table = [];

            % cycle over upper triagle of naxna matrix (all pairs)
            for i = 1:(na - 1)

                % no diagonal
                ma = i + 1;

                % get location and ID of Ai
                i_loc = agents{i}.location;
                i_num = agents{i}.agent_number;

                % second cycle
                for j = ma:na

                    % get location and ID of Aj
                    j_loc = agents{j}.location;
                    j_num = agents{j}.agent_number;

                    % store IDs and locs
                    los_table(end+1,:) = [i_num,j_num,i_loc,j_loc];
                end

            end

        end

        % draw LOS on the map
        function draw_los_map(obj)

            % call static method
            Map.set_figure();

            % compute LOS distances
            los_table = obj.calc_los_map();

            % get number of pairs and get Ni (rows)
            n = size(los_table);
            n = n(1);

            % for each Ai draw LOS
            for i = 1:n

                % get locs
                c_line = los_table(i,:);

                % get origin and endpoint in XoY
                x1 = c_line(3);
                x2 = c_line(5);
                y1 = c_line(4);
                y2 = c_line(6);

                % plot
                plot([x1,x2],[y1,y2],'Color','k');
            end

        end

    end
    
end