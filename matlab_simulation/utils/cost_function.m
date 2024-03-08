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
    % JR = 1/(1 + min(e));    
    % JR = min(e)/max(e);

    % is connected
    % A = calcAdjacencyMatrix(los_table,agents_list);
    % [allConn, A] = agentsOpt{1}.checkConnectivity(A);

           

    % get agents_pos
    % agents_pos = agents_list(:,2:3);
    % sigma = std(agents_pos,1);
    % sigmacost = sigma*sigma';
    % JS = 1/(1+sigmacost);
    if ~isempty(los_table)
        JS = 0;
        N = size(los_table,1);  

        % spreadiness term
        tmp = los_table(:,5:5+p-1) - los_table(:,5+p:5+2*p-1);
        JS = JS + sum(tmp(:,1).^2.*tmp(:,2).^2);    
        JS = -JS/N^6;

        % symmetry term        
    else
        JS = 0;
    end
    

    % penalty on Dmin
    % if ~isempty(los_table)
    %     D = los_table(:,5:5+p-1) - los_table(:,5+p:5+2*p-1);
    %     Dmin = min(vecnorm(D,2,2));
    % else
    %     Dmin = 0;
    % end
    % JD = 1/(1+Dmin);
    JD = 0;
    

    % normalize
    % minterm = min(JD,JR);    
    J = 1*JR + 0*JD + 0*JS;

    if isempty(J)
        J = Inf;
    end

end