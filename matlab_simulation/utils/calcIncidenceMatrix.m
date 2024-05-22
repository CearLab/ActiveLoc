%% Calc incidence matrix
% file: calcIncidenceMatrix.m
% author: Ido Sherf 
% date: 01/02/2024
% description: calculate the incidence matrix for given los table 
function I = calcIncidenceMatrix(los_table,agents_list)

    n = size(agents_list,1); % there is |V| row in I
    m = size(los_table,1); % there is |E| col in I
    I = zeros(n,m);

    % cycle over the edges
    for  ie = 1:m

        % get edge 
        e = los_table(ie,:);

        % get agents ID
        v1 = e(1);
        v2 = e(2);

        % assign ones in node row
        I(v1,ie) = 1;
        I(v2,ie) = 1;
        
    end

end

