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
        function setFigure()

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
        function addPolygon(obj,geo)

            % call the obstacle method (see Obstacle class)
            obj.obs_list{end + 1} = Obstacle('polygon',geo);

        end

        % plot on figure all obstacles
        function drawAllObs(obj)

            % get number of ostacles
            n = numel(obj.obs_list);

            % call static method
            Map.setFigure();

            % set environment limits from class property
            xlim(obj.map_span(1,:));ylim(obj.map_span(2,:))

            % draw the obstacles (see Obstacle class)
            for i = 1:n
                obj.obs_list{i}.drawOnMap()
            end

        end

        % plot all teams of heterogeneous objects
        function drawAllTeams(obj)

            % get the Agent managers
            manager = AgentManager.getInstance();

            % get list of teams
            team_list = manager.getAllTeams();

            % get number of teams
            n = numel(team_list);

            % call static method
            Map.setFigure();

            % set environment limits from class property
            xlim(obj.map_span(1,:));ylim(obj.map_span(2,:))

            % draw the team (see team class)
            for i = 1:n
                team_list{i}.plotTeam();
            end

        end

    end
end