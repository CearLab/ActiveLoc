%% Nonlinear constraints
% file: nonlcon.m
% author: Federico Oliva 
% date: 01/02/2024
% description: cost function constraints
function [c, ceq] = nonlcon(x,p)

    c = [];
    ceq = [];

    % the x is a vector with all the agents positions stacked in a columnt
    % p is the space dimension

    % reshape agents positions
    xmat = reshape(x,p,floor(numel(x)/p))';

    % get the manager
    manager = AgentManager.getInstance();

    % get the team you just created
    team = manager.team_list{2};    
    
    % get all agents from the team 
    agents0 = {team.team_mates{1:end}};    
    % get m
    m = manager.WS.m;
    % insert leader in right position
    agents0 = {agents0{1:team.leader.agent_number-1-m} team.leader agents0{team.leader.agent_number-m:end}};

    % get agents summary tables
    [los_table0,agents_list0] = calcLosMap(agents0,'UWB');

    % get incidence matrix
    I0 = calcIncidenceMatrix(los_table0,agents_list0);

    % get the team you just created
    team = manager.team_list{1};    
    
    % get all agents from the team 
    agents = {team.team_mates{1:end}};        
    % insert leader in right position
    agents = {agents{1:team.leader.agent_number-1} team.leader agents{team.leader.agent_number:end}};

    % reassign agents position
    for i=1:numel(agents)
        agents{i}.location = xmat(i,:);
    end

    % get agents summary tables
    [los_table,agents_list] = calcLosMap(agents,'UWB');    
    

    % I also want to ensure a min distance between agents
    % get distances
    if ~isempty(los_table)
        D = los_table(:,5:5+p-1) - los_table(:,5+p:5+2*p-1);
        Dmin = min(vecnorm(D,2,2));
        Dmax = max(vecnorm(D,2,2));
    else
        Dmin = 0;
        Dmax = 0;
    end

    % set Dminthresh
    Dminthresh = manager.WS.Dminthresh;
    if isinf(Dminthresh)
        Dminthresh = 0;
    end    

    % set Dmaxthresh
    Dmaxthresh = manager.WS.Dmaxthresh;
    if isinf(Dmaxthresh)
        Dmaxthresh = 1e2;
    end

    % set constraint
    distconmin = (Dminthresh - Dmin);
    distconmax = (Dmax - Dmaxthresh);

    % of course I want to keep rigidity
    % get rigidity matrix 
    R = calcRigitdyMatrix(los_table,agents_list);

    % get nonzero eigs
    e = eig(R'*R);

    % get # nnz elements
    pos = find(abs(e) < manager.WS.eigThresh);
    isrigid = ~(numel(pos)==3);

    [allConn, ~] = checkKconnectivity(los_table,agents_list,3);
    isConn = ~allConn;    

    % equality constraints
    ceq = [ceq; isrigid];
    ceq = [ceq; isConn];    

    % inequality constraints
    c = [c; distconmin];
    c = [c; distconmax];
    % c = [c; rho2conmin];
    % c = [c; InsideCircle];




end