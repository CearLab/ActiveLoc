%% calcLosMap
% file: calcLosMap.m
% author: Ido Sherf 
% date: 01/02/2024
% description: calculate the los table for the input agent list
function [los_table,agents_list] = calcLosMap(agents,sensorName)

    % get number of agents
    na = numel(agents);
    
    % init empty list
    agents_list = [];   % this contains all the info on agents involved in LOS
    los_table = [];     % this contains aal the info of agents' pairs in LOS
    
    % cycle the agents and define agents_list
    for i = 1:na
        agents_list(end + 1,:) = [agents{i}.agent_number, agents{i}.location];
    end
    
    % cycle over upper triagle of naxna matrix (all pairs)
    for i = 1:(na - 1)
    
        % no diagonal
        ma = i + 1;
    
        % get location and ID of Ai (owner)
        i_loc = agents{i}.location;
        i_loc_est = agents{i}.location_est;
        i_num = agents{i}.agent_number;
    
        % second cycle
        for j = ma:na
    
            % get location and ID of Aj (targets)
            j_loc = agents{j}.location;
            j_loc_est = agents{j}.location_est;
            j_num = agents{j}.agent_number;
    
            % check measure policy
            cmd = ['[~, meas_exist] = agents{i}.sensors.',sensorName,'.RangeSensorPolicy(i_loc,j_loc);'];            
            try                
                eval(cmd);
            catch
                try
                    cmd = ['[~, meas_exist] = agents{j}.sensors.',sensorName,'.RangeSensorPolicy(j_loc,i_loc);'];
                    eval(cmd);
                catch
                    meas_exist = 0;
                end                
            end
    
            % sensor info
            if strcmp(sensorName,'UWB')
                sens = 1;
            elseif strcmp(sensorName,'CAM')
                sens = 2;
            else
                sens = -1;
            end
            if meas_exist
                % store IDs and locs
                los_table(end+1,:) = [i,j,i_num,j_num,i_loc,j_loc,sens];
            end

        end
    
    end

end