%% calcLosMap
% file: calcLosMap.m
% author: Ido Sherf 
% date: 01/02/2024
% description: calculate the los table for the input agent list
function [los_table,agents_list] = calcLosMap(agents)

% get number of agents
na = numel(agents);

% init emptu list
agents_list = [];
los_table = [];
for i = 1:na
    agents_list(end + 1,:) = [agents{i}.agent_number, agents{i}.location];
end
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
        [~, meas_exist] = Sensor.RangeSensorPolicy(i_loc,j_loc);
        if meas_exist
            % store IDs and locs
            los_table(end+1,:) = [i,j,i_num,j_num,i_loc,j_loc];
        end
    end

end

end