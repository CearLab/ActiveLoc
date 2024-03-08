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
        v1 = e(3);
        v2 = e(4);
        A(v1,v2) = 1;
        A(v2,v1) = 1;
        
    end

    % we need to remove the useless columns    
    IDpres = agents_list(:,1);
    maxID = max(IDpres);
    IDtot = 1:maxID;

    % now remove all IDs that are not in the mesh
    IDloss = IDtot;
    for i=1:numel(IDpres)
        pos = find(IDloss == IDpres(i));
        IDloss(pos) = [];
    end

    % delete columns
    cols = IDloss;
    A(:,cols) = [];    
    A(cols,:) = [];

    A = A + eye(n);
end

