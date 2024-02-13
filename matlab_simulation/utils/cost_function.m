%% Cost function
% file: cost_function.m
% author: Federico Oliva 
% date: 01/02/2024
% description: cost function to optimize 
function J = cost_function(x,p)

    % the x is a vector with all the agents positions stacked in a columnt
    % p is the space dimension

    % reshape agents positions
    xmat = reshape(x,p,floor(numel(x)/p))';

    % get the manager
    manager = AgentManager.getInstance();

    % get teams
    teams = manager.getAllTeams();

    % get the agents
    agents = teams{2}.team_mates;

    % reassign agents position
    for i=1:numel(agents)
        agents{i}.location = xmat(i,:);
    end

    % get agents summary tables
    [los_table,agents_list] = calcLosMap(agents);

    % get rigidity matrix 
    R = calcRigitdyMatrix(los_table,agents_list);

    % get nonzero eigs
    e = eig(R'*R);
    pos = find(abs(e) < 1e-10);
    e(pos) = [];
    
    J = 1/min(e);
    % J = max(e)/min(e);

    if ~isscalar(J)
        J = Inf;
    end

    % store cost function
    manager.WS.J(end+1) = J;

    % store condition number
    manager.WS.CN(end+1) = max(e)/min(e);

    % store optimized var
    manager.WS.X(:,end+1) = x;

end