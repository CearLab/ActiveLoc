%% calc adjacency matrix
% file: calcDistanceMatrix.m
% author: Federico Oliva 
% date: 15/02/2024
% description: calculate the adjacency matrix for given los table, but with
% distances instead of booleans
function A = calcDistanceMatrix(los_table,agents_list)

    % get number of agents
    n = length(agents_list); % A is a |V|*|V| matrix

    % init matrix
    A = zeros(n);

    % cycle over the los table
    for ie = 1:size(los_table,1)

        % get ids
        e = los_table(ie,:);
        v1 = e(1);
        v2 = e(2);
        
        s = Sensor(v1);

        % get distances
        A(v1,v2) = s.getRangeMeas(v2);
        A(v2,v1) = A(v1,v2);
    end

end

