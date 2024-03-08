%% calc rigitdy matrix
% file: calcRigitdyMatrix.m
% author: Ido Sherf 
% date: 01/02/2024
% description: calculate the rigitdy matrix for given los table 
function R = calcRigitdyMatrix(los_table,agents_list)

    % define where we work (plane)
    p = 2; 
    
    n = size(los_table,1); % there is |E| row in R
    
    m = size(agents_list,1)*p; % there is p*|V| cols in R
    
    % init
    R = zeros(n,m);

    % if less than 3 agents no rigid
    if n < 3        
        return;
    end
    
    % cycle over edges
    for  ie = 1:n
    
        % get LOS
        e = los_table(ie,:);
    
        % get indices
        v1 = e(3);
        v2 = e(4);
    
        % get diff pos
        p1 = e(5:5+p-1);
        p2 = e(5+p:5+2*p-1);        
    
        % terms
        p1p2 = p1-p2;
        p2p1 = -p1p2;
    
        % positions in the matrix
        v1_r_idx = (v1-1)*2 + 1 : (v1-1)*2 + p;
        v2_r_idx = (v2-1)*2 + 1 : (v2-1)*2 + p;
    
        % set values
        R(ie,v1_r_idx) = p1p2;
        R(ie,v2_r_idx) = p2p1;
    
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
    cols = sort([p*IDloss-1 , p*IDloss]);
    R(:,cols) = [];    

end

