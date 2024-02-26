%% calc adjacency matrix
% file: calcIncidenceMatrix.m
% author: Ido Sherf 
% date: 01/02/2024
% description: calculate the adjacency matrix for given los table 
function A = calcAdjacencyMatrix(los_table,agents_list)

    n = length(agents_list); % A is a |V|*|V| matrix
    A = zeros(n);
    
    for ie = 1:size(los_table,1)

        e = los_table(ie,:);
        v1 = e(1);
        v2 = e(2);
        A(v1,v2) = 1;
        A(v2,v1) = 1;
        
    end
end

