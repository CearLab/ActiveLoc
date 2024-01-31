%% Agent class
% file: Agent.m
% author: Ido Sherf 
% date: 22/01/2024
% description: hndle class handling the creation of an agent on a Map 
% (see map class)
classdef Agent <handle

    % class properties
    properties

        % agent ID
        agent_number

        % position on the plane
        location

        % role?
        roll

        % team (maybe here I would return just the ID instead of all the team)
        team

    end

    % class methods
    methods

        % class constructor
        function obj = Agent(agent_number,location,team,roll)

            % arguments validatio
            arguments

                agent_number
                location
                team
                roll = 'team_mate'

            end

            % assign values
            obj.agent_number = agent_number;
            obj.team = team;
            obj.location = location;
            obj.roll = roll;

        end

    end

end