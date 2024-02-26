%% calc adjacency matrix
% file: calcDistanceMatrix.m
% author: Federico Oliva 
% date: 15/02/2024
% description: calculate the adjacency matrix for given los table, but with
% distances instead of booleans
function E = StrutEnergy(X)

    % get the manager
    manager = AgentManager.getInstance();
    p = manager.WS.p;
    N = numel(manager.team_list{1}.team_mates) + numel(manager.team_list{1}.leader);
    P = X(1:p*N);
    w = X(p*N+1:end);

    % reshape agents positions
    xmat = reshape(P,p,floor(numel(P)/p))';    

    W = manager.WS.W0;
    [r, c] = find(W ~= 0);
    for i=1:numel(r)
        W(r(i),c(i)) = w(i);
    end
    
    % init energy
    E = zeros(p,1);

    % cycle only on the upper triangle of Distance Matrix
    for i=1:numel(r)        
        D = (xmat(r(i),:) - xmat(c(i),:))';
        E = E + W(r(i),c(i))*D.^2;
    end    

    E = norm(E);

end