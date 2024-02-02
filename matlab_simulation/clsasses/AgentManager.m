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
        team_list = {}

        %
        agent_pointer_list = {}

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

        % reset lists
        function reset(obj)
            obj.team_list = {};
            obj.agent_pointer_list = {};
            obj.agents_counter = 0;
        end

        % create an agent using Agent class
        function createAgent(obj, location, team_id, role)

            % get team ID
            team = obj.getOrCreateTeam(team_id);

            % increase number of agents 
            obj.agents_counter = obj.agents_counter + 1;

            % call constructor for Agent class
            agent = Agent(obj.agents_counter,location,team_id,role);
            
            %add the agent to the agent list
            
            obj.agent_pointer_list{obj.agents_counter} = agent;

            % check on agent role
            if strcmp(role,'team_mate')

                % add agent to the team (see Team class)
                team.addTeamMate(agent)

            elseif strcmp(role,'team_leader')

                % set team leader (see Team class)
                team.setLeader(agent)
            else

                % raise error
                error('role should be team_mate or team_leader')
            end
        end

        % create a new team with ID
        function team = addTeam(obj,team_id)

            % call constructor from Team class
            team = Team(team_id);

            % add item to team list
            obj.team_list{end+1} = team;
        end

        % add team to the list if there is no team. If not, get the team
        % handle. Maybe the name here is not so straightforward
        function team = getOrCreateTeam(obj,team_id)

            % if there is no team, create one with ID
            if isempty(obj.team_list)

                % call class method
                team = obj.addTeam(team_id);
                return

            end

            % if teams already exist, get the handle from team ID
            % cycle over team IDs. 
            for t = 1:length(obj.team_list)

                % get the right one
                if obj.team_list{t}.team_number == team_id

                    % return team handle
                    team = obj.team_list{t};
                    return
                end

            end

            % if no team matches the number, create a new team
            team = obj.addTeam(team_id);
        end

        % get all teams (why this if the property is public and there is 
        % only an AgentManager?)
        function teams_list = getAllTeams(obj)

            % access class property
            teams_list = obj.team_list;

        end

        % get all agents (see Team class)
        function agents = getAllAgent(obj)
            %return all the agents
            agents = obj.agent_pointer_list;
        end

    end

end