classdef Agent <handle

    properties
        agent_number
        location
        roll
        team
    end
    methods
        function obj = Agent(agent_number,location,team,roll)
            arguments
                agent_number
                location
                team
                roll = 'team_mate'
            end
            obj.agent_number = agent_number;
            obj.team = team;
            obj.location = location;
            obj.roll = roll;
        end

    end
end