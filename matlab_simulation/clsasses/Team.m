classdef Team < handle
    
    properties
        team_number
        leader 
        team_mates = {};
    end
    
    methods
        function obj = Team(team_number)
            obj.team_number = team_number;
        end
        function obj = set_leader(obj,leader)
            obj.leader = leader;
        end
        function add_team_mate(obj,agent)
            obj.team_mates{end +1} = agent;
        end
        function agents = get_all_agents(obj)
            agents = {};
            if ~isempty(obj.leader)
                agents{end + 1} = obj.leader;
            end
            if ~isempty(obj.team_mates)
                agents = [agents, obj.team_mates];
            end
        end
        function plot_team(obj)
            col = get_color_by_number(obj.team_number);
            loc = [];
            for i = 1:numel(obj.team_mates)
                loc(end+1,:) = obj.team_mates{i}.location;
            end
            plot(loc(:,1),loc(:,2),'Color',col, ...
                'LineStyle','none', ...
                'Marker','o', ...
                'MarkerSize',12,'MarkerFaceColor',col)
            team_lead_loc = obj.leader.location;
            plot(team_lead_loc(:,1),team_lead_loc(:,2),'Color',col, ...
                'LineStyle','none', ...
                'Marker','diamond', ...
                'MarkerSize',12,'MarkerFaceColor',col)
        end
    end
end

