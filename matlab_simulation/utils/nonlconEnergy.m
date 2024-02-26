%% Nonlinear constraints
% file: nonlcon.m
% author: Federico Oliva 
% date: 01/02/2024
% description: cost function constraints
function [c, ceq] = nonlconEnergy(x,p)

    c = [];
    ceq = [];

    % the x is a vector with all the agents positions stacked in a columnt
    % p is the space dimension

    % reshape agents positions
    xmat = reshape(x,p,floor(numel(x)/p))';

    % get the manager
    manager = AgentManager.getInstance();

    % get teams
    teams = manager.getAllTeams();

    % get the agents of the initial condition
    agents0 = teams{1}.team_mates;

    % get agents summary tables
    [los_table,agents_list] = calcLosMap(agents0);

    % I also want to ensure a min distance between agents
    % get distances
    if ~isempty(los_table)
        D = los_table(:,5:5+p-1) - los_table(:,5+p:end);
        Dmin = min(vecnorm(D,2,2));
        Dmax = max(vecnorm(D,2,2));
    else
        Dmin = 0;
        Dmax = Inf;
    end

    % set Dminthresh
    Dminthresh = 0.5*Sensor.max_range;
    if isinf(Dminthresh)
        Dminthresh = 0;
    end        

    % set constraint
    distconmin = (Dminthresh - Dmin);   

    % of course I want to keep rigidity
    % get rigidity matrix 
    R = calcRigitdyMatrix(los_table,agents_list);

    % get nonzero eigs
    e = eig(R'*R);

    % get # nnz elements
    pos = find(abs(e) < 1e-10);
    isrigid = ~(numel(pos)==3);

    % circular area constraint
    map = Map.getInstance();
    Rmax = 1*max(map.map_span(:,2));
    R = vecnorm(xmat,2,2);
    InsideCircle = R - Rmax;

    % equality constraints
    ceq = [ceq; isrigid];    

    % inequality constraints
    c = [c; distconmin];
    c = [c; InsideCircle];




end