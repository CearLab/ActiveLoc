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

    % get number of agents
    m = manager.WS.m;

    % get the team you just created
    team = manager.team_list{1};    
    
    % get all agents from the team 
    agents = {team.team_mates{1:end}};    
    % insert leader in right position
    agents = {agents{1:team.leader.agent_number-1} team.leader agents{team.leader.agent_number:end}};


    % reassign agents position
    for i=1:m
        agents{i}.location = xmat(i,:);
    end

    % get agents summary tables
    [los_table,agents_list] = calcLosMap(agents,'UWB');

    % get rigidity matrix 
    R = calcRigitdyMatrix(los_table,agents_list);

    % get nonzero eigs
    e = eig(R'*R);
    pos = find(abs(e) < 1e-10);
    e(pos) = [];
    
    J = 1/min(e);
    % J = min(e)/max(e);

    if ~isscalar(J)
        J = Inf;
    end

    % store cost function
    manager.WS.J(end+1) = J;

    % store condition number
    if ~isempty(e)
        manager.WS.CN(end+1) = max(e)/min(e);
    end

    % store optimized var
    manager.WS.X(:,end+1) = x;

end