%% Sensor class
% file: Sensor.m
% author: Ido Sherf 
% date: 31/01/2024
% description: hndle class handling the sensors assigned to agents
classdef Sensor < handle
    properties
        sensor_type = 'range'
        owner
    end
    properties(Constant)
        max_range = inf;
    end

    methods(Static)
        % this function return the distance meas if the meas is valid 
        % and NaN if its not
        function [distance, meas_exist] = RangeSensorPolicy(owner_location, target_location)
            % inforce maximum range
            distance = norm(target_location - owner_location);
            meas_exist = true;
            if distance > Sensor.max_range
                distance = NaN;
                meas_exist = false;
            end

            % TODO: NLOS calculation goes here
        end
    end
    methods
        function obj = Sensor(owner,type)
           arguments
               owner
               type = 'range';
           end
           obj.sensor_type = type;
           obj.owner = owner;

        end
        function meas = getRangeMeas(obj,target_agent_id)
            % get the agents objects
            manager = AgentManager.getInstance();
            target_agent = manager.agent_pointer_list{target_agent_id};
            owner_agent = manager.agent_pointer_list{obj.owner};
            % get the distance
            target_location = target_agent.location;
            owner_location = owner_agent.location;
            % calc the actual measerment
            meas = RangeSensorPolicy(target_location,owner_location);
        end
    end
end

