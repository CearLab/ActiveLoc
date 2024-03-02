%% Nonlinear constraints
% file: nonlcon.m
% author: Federico Oliva 
% date: 01/02/2024
% description: cost function constraints
function [c, ceq] = nonlconEnergy(x,p)

    c = [];
    ceq = [];

    % the x is a vector with all the agents positions stacked in a column
    % p is the space dimension
    % c are the ineqiality constr
    % ceq the equality constr
    % trying to satisfy c < 0 and ceq == 0

    % reshape agents positions in a Nxp matrix
    xmat = reshape(x,p,floor(numel(x)/p))';

    % get the manager
    manager = AgentManager.getInstance();

    % get teams
    teams = manager.getAllTeams();

    % get the agents of the team #1, which is the team storing the initial
    % condition
    agents0 = {teams{1}.team_mates{1:end}};
    agents0 = {agents0{1:teams{1}.leader.agent_number-1} teams{1}.leader agents0{teams{1}.leader.agent_number:end}};    

    % get again the LOS table (could have passed it as @param)
    [los_table,agents_list] = calcLosMap(agents0,'UWB');

    % I also want to ensure a min distance between agents    
    if ~isempty(los_table)
        % get distances
        D = los_table(:,5:5+p-1) - los_table(:,5+p:5+2*p-1);

        % find max and min dist
        Dmin = min(vecnorm(D,2,2));
        Dmax = max(vecnorm(D,2,2));

    else
        % if no connection is present, only reasonable limits
        Dmin = 0;
        Dmax = Inf;

    end

    % set Dminthresh (min distance threshold)
    Dminthresh = 0.1*teams{1}.leader.sensors.UWB.max_range;
    if isinf(Dminthresh)
        Dminthresh = 0;
    end        

    % set constraint (c < 0)
    distconmin = (Dminthresh - Dmin);   

    % of course I want to keep rigidity, if it is present
    % get rigidity matrix 
    R = calcRigitdyMatrix(los_table,agents_list);

    % get nonzero eigs
    e = eig(R'*R);

    % get # nnz elements
    pos = find(abs(e) < 1e-10);

    % boolean flag on lambda4
    isrigid = ~(numel(pos)==3);

    % circular area constraint (useless now)
    map = Map.getInstance();
    Rmax = 1*max(map.map_span(:,2));
    R = vecnorm(xmat,2,2);
    InsideCircle = R - Rmax;

    % equality constraints (I want rigidity)
    ceq = [ceq; isrigid];    

    % inequality constraints
    c = [c; distconmin];        % min distance
    % c = [c; InsideCircle];




end