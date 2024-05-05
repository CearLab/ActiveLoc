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
    leaderID = teams{1}.leader.agent_number;

    for i=1:numel(agents0)
        X0(i,:) = agents0{i}.location;
    end

    % leader keep the position
    FixLeader = ~(sum(X0(leaderID,:) == xmat(leaderID,:)) == 2);

    % bars constraints: the distances of the bars must be always the same as in
    % Dslam
    for i = 1:size(manager.WS.bars,1)
        deltaPbars(i,:) = xmat(manager.WS.bars(i,1),:) - xmat(manager.WS.bars(i,2),:);
    end
    dbars = manager.WS.Dbars(manager.WS.bars(1,1),manager.WS.bars(:,2))';
    FixBars = ~(sum(deltaPbars*deltaPbars' == dbars*dbars')==size(manager.WS.bars,1)); 



    % equality constraints (I want rigidity)
    % ceq = [ceq; FixLeader];    

    % inequality constraints
    c = [c; FixLeader   -1e-1]; 
    % c = [c; FixBars     -1e-1]; 




end