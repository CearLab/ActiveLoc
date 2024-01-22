classdef AgentManager < handle
    properties (Access = public)
        TeamList = {}
        agents_counter = 0
    end

    methods (Access = private)
        function obj = AgentManager()
            % Private constructor for singleton
        end
    end

    methods (Static)
        function singleObj = getInstance()
            persistent uniqueInstance
            if isempty(uniqueInstance)
                uniqueInstance = AgentManager();
            end
            singleObj = uniqueInstance;
        end
    end

    methods
        function createAgent(obj, location, team_num, roll)
            team = obj.getTeam(team_num);
            obj.agents_counter = obj.agents_counter + 1;
            agent = Agent(obj.agents_counter,location,team,roll);
            if strcmp(roll,'team_mate')
                team.add_team_mate(agent)
            elseif strcmp(roll,'team_leader')
                team.set_leader(agent)
            else
                error('roll should be team_mate or team_leader')
            end
        end
        function team = getTeam(obj,team_num)
            if isempty(obj.TeamList)
                team = obj.add_team(team_num);
                return
            end
            for t = 1:length(obj.TeamList)
                if obj.TeamList{t}.team_number == team_num
                    team = obj.TeamList{t};
                    return
                end
            end
            team = obj.add_team(team_num);
        end
        function team = add_team(obj,team_num)
            team = Team(team_num);
            obj.TeamList{end+1} = team;
        end
        function teams_list = get_all_teams(obj)
            teams_list = obj.TeamList;
        end
        function agents = get_all_agent(obj)
            teams = obj.get_all_teams();
            n = numel(teams);
            agents = {};
            for i = 1:n
                agent_list = teams{i}.get_all_agents();
                agents = [agents,agent_list];
            end
            
        end
    end
end