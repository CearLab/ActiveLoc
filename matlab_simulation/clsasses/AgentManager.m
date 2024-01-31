%% Agent manager class
% file: AgentManager.m
% author: Ido Sherf 
% date: 22/01/2024
% description: hndle class handling the creation of an agent manager for 
% all teams on a Map. Specifically, this class is a coded as 
% singleton, namely it is ensured that the class has only one instance and 
% it is provided a way to access that instance from any point in the 
% application. 
% (see map class)
classdef AgentManager < handle

    % class properties (public)
    properties (Access = public)

        % list of teams
        TeamList = {}

        % counter
        agents_counter = 0

    end

    % class methods (private)
    methods (Access = private)

        % Private constructor for singleton
        function obj = AgentManager()
        end

    end

    % class methods (static)
    methods (Static)

        % get a single class instance from everywhere in the application
        function singleObj = getInstance()

            % define persistent variable (still exists after the call)
            persistent uniqueInstance

            % if it is empty, call the constructor
            if isempty(uniqueInstance)
                uniqueInstance = AgentManager();
            end

            % assign the access point to the persistent var
            singleObj = uniqueInstance;

        end

    end

    % class methods
    methods

        % create an agent using Agent class
        function createAgent(obj, location, team_num, role)

            % get team ID
            team = obj.getTeam(team_num);

            % increase number of agents 
            obj.agents_counter = obj.agents_counter + 1;

            % call constructor for Agent class
            agent = Agent(obj.agents_counter,location,team,role);

            % check on agent role
            if strcmp(role,'team_mate')

                % add agent to the team (see Team class)
                team.add_team_mate(agent)

            elseif strcmp(role,'team_leader')

                % set team leader (see Team class)
                team.set_leader(agent)
            else

                % raise error
                error('role should be team_mate or team_leader')
            end
        end

        % create a new team with ID
        function team = add_team(obj,team_num)

            % call constructor from Team class
            team = Team(team_num);

            % add item to team list
            obj.TeamList{end+1} = team;
        end

        % add team to the list if there is no team. If not, get the team
        % handle. Maybe the name here is not so straightforward
        function team = getTeam(obj,team_num)

            % if there is no team, create one with ID
            if isempty(obj.TeamList)

                % call class method
                team = obj.add_team(team_num);
                return

            end

            % if teams already exist, get the handle from team ID
            % cycle over team IDs. 
            for t = 1:length(obj.TeamList)

                % get the right one
                if obj.TeamList{t}.team_number == team_num

                    % return team handle
                    team = obj.TeamList{t};
                    return
                end

            end

            % if no team matches the number, create a new team
            team = obj.add_team(team_num);
        end

        % get all teams (why this if the property is public and there is 
        % only an AgentManager?)
        function teams_list = get_all_teams(obj)

            % access class property
            teams_list = obj.TeamList;

        end

        % get all agents (see Team class)
        function agents = get_all_agent(obj)

            % get all teams from class method
            teams = obj.get_all_teams();

            % get number of teams
            n = numel(teams);

            % init empty cell array
            agents = {};

            % cycle over the teams
            for i = 1:n

                % for each team get all agents
                agent_list = teams{i}.get_all_agents();

                % append agents
                agents = [agents,agent_list];

            end
            
        end

    end

end