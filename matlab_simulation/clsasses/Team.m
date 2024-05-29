%% Team class
% file: Team.m
% author: Ido Sherf
% date: 22/01/2024
% description: handle class handling the creation of a Team manager from a
% set of agents on a Map.
classdef Team < handle

    % class properties
    properties

        % team ID
        team_number

        % leader (Agent)
        leader

        % cell array containing the teammates
        team_mates = {};

        % Rigidity matrix
        rigidity_matrix = [];



    end

    % class methods
    methods

        % class constructor
        function obj = Team(team_number)

            % assign team ID
            obj.team_number = team_number;

        end

        % set the leader status
        function obj = setLeader(obj,leader)

            % assign leader agent
            obj.leader = leader;

        end

        % add teammate to the team
        function addTeamMate(obj,agent)

            % append agent to the teammates array
            obj.team_mates{end +1} = agent;

        end

        % get all agents of a team
        function agents = getAllAgents(obj)

            % init cell array
            agents = {};

            % if there is a leader, add it to the list (so there can be a
            % team without a leader?)
            if ~isempty(obj.leader)

                % append leader
                agents{end + 1} = obj.leader;

            end

            % if there are teammates add them to the list
            if ~isempty(obj.team_mates)

                % append teammates
                agents = [agents, obj.team_mates];

            end

        end

        % plot team
        function plotTeam(obj,varargin)            

            % init empty list of locations
            loc = [];
            ID = [];

            % cycle over the number of teammates
            for i = 1:numel(obj.team_mates)

                % get locations
                loc(end+1,:) = obj.team_mates{i}.location;

                % ID
                ID(end+1,:) = obj.team_mates{i}.agent_number;

            end
            map = Map.getInstance();

            % if no figure handle provided call map
            if isempty(varargin)
                map.setFigure();
            else
                try
                    figure(varargin{1});
                catch                    
                    axes(varargin{1});
                end

                % call external function
                if numel(varargin) == 2
                    col = getColorByNumber(varargin{2});
                else                    
                    col = getColorByNumber(obj.team_number);
                end 
            end

            % cheating
            if ~isempty(obj.leader)
                tmp = 1;
            else
                tmp = 0;
            end
            if min(ID) > numel(ID)
                ID = ID - numel(ID)-tmp;
            end
            
            if ~isempty(loc)
                % plot all teammates + graphic info
                % plot(loc(:,1),loc(:,2),'Color',col, ...
                %     'LineStyle','none', ...
                %     'Marker','o', ...
                %     'MarkerSize',12,'MarkerFaceColor',col);

                text(1*loc(:,1),1*loc(:,2), ...                    
                    cellstr(num2str(ID)), ...                    
                    'FontSize',20, ...
                    'Color',col);
            end
            if ~isempty(obj.leader)
                % get leader location
                team_lead_loc = obj.leader.location;

                % plot leader + graphic info
                plot(team_lead_loc(:,1),team_lead_loc(:,2),'Color',col, ...
                    'LineStyle','none', ...
                    'Marker','diamond', ...
                    'MarkerSize',12,'MarkerFaceColor',col);
            end

        end



    end
end


