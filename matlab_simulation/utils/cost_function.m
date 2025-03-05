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
    teamOpt = manager.team_list{1};
    
    % get all agents from the team optimized 
    agentsOpt = {teamOpt.team_mates{1:end}};
    agentsOpt = {agentsOpt{1:teamOpt.leader.agent_number-1} teamOpt.leader agentsOpt{teamOpt.leader.agent_number:end}};


    % reassign agents position
    for i=1:m
        agentsOpt{i}.location = xmat(i,:);
    end

    % get agents summary tables
    [los_table,agents_list] = calcLosMap(agentsOpt,'UWB');

    % get rigidity matrix 
    R = calcRigitdyMatrix(los_table,agents_list);    

    % J rigidity   
    e = eig(R'*R);    
    pos = find(abs(e) < 1e-5);    
    isrigid = (numel(pos)==3);              
    e(pos) = [];
    JR = -min(e);
    % JR = 1/(min(e));        

    % Jtest
    Rover = R'*R;
    tr = trace(Rover);
    JT = -tr;

    % eq. (30)

    % getNeighbors
    for i=1:size(agents_list,1)
        agentsOpt{i}.getNeighbors;
    end

    % mean
    mu_x = mean(agents_list(:,2));
    mu_y = mean(agents_list(:,3));

    % terms computation
    for i=1:size(agents_list,1)

        % neighborhood cardinality
        Ni(i) = size(agentsOpt{i}.neigh.ID,1);

        % term 1
        T1_x(i) = Ni(i)*(agents_list(i,2)-mu_x)^2;
        T1_y(i) = Ni(i)*(agents_list(i,3)-mu_y)^2;

        % term 2
        T2_x(i) = 0;
        T2_y(i) = 0;
        for j=1:size(agentsOpt{i}.neigh.ID,1)
            T2_x(i) = T2_x(i) + (agents_list(agentsOpt{i}.neigh.ID(j),2) - mu_x)^2;
            T2_y(i) = T2_y(i) + (agents_list(agentsOpt{i}.neigh.ID(j),3) - mu_y)^2;
        end

        % term 3        
        T3_x(i) = 0;
        T3_y(i) = 0;
        for j=1:size(agentsOpt{i}.neigh.ID,1)
            T3_x(i) = T3_x(i) + agents_list(agentsOpt{i}.neigh.ID(j),2);
            T3_y(i) = T3_y(i) + agents_list(agentsOpt{i}.neigh.ID(j),3);
        end
        T3_x(i) = -2*(agents_list(i,2)-mu_x)*(T3_x(i) - Ni(i)*mu_x);
        T3_y(i) = -2*(agents_list(i,3)-mu_y)*(T3_y(i) - Ni(i)*mu_y);

    end

    % store
    manager.WS.T1_x = T1_x;
    manager.WS.T1_y = T1_y;
    manager.WS.T2_x = T2_x;
    manager.WS.T2_y = T2_y;
    manager.WS.T3_x = T3_x;
    manager.WS.T3_y = T3_y;
    

    % normalize
    % minterm = min(JD,JR);    
    % J = 1*JR + 0*JT;
    J = -(0*sum(T1_x) + 0*sum(T1_y) + 0*sum(T2_x) + 0*sum(T2_y) + 1*sum(T3_x) + 1*sum(T3_y));    

    if isempty(J)
        J = Inf;
    end

end