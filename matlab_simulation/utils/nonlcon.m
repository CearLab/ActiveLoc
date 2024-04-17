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

    A = calcAdjacencyMatrix(los_table,agents_list);
    [allConn, A] = agents{1}.checkConnectivity(A);
    isConn = ~allConn;

    % circular area constraint
    map = Map.getInstance();
    Rmax = 1*max(map.map_span(:,2));
    R = vecnorm(xmat,2,2);
    InsideCircle = R - Rmax;

    %%% symmetry    
    rho2min = 0.5;
    if ~isempty(los_table)

        % init
        JS = 0;        

        % init                                
        SXYtmp = 0;
        S2tmp = 0;
        S2T1 = 0;
        S2T2 = 0;        

        % cycle over all the agents
        for a=1:m

            % init possible deltas
            LOCb = [];
            POSb = [];

            % get position of single agent
            LOCa = agents_list(a,2:3);

            % now inner cycle
            % I'm not removing a~=b because the diff is zero anyways
            for b=1:m
                
                % now cycle over the LOS tab                

                % find if the link does exist
                flagA = los_table(:,3:4) == [a b];
                flagB = los_table(:,3:4) == [b a];
                posA = find(flagA(:,1).*flagA(:,2) == 1);
                posB = find(flagB(:,1).*flagB(:,2) == 1);      
                flag = min([posA posB]); 

                % assign
                if ~isempty(flag)
                    LOCb(b,:) = LOCa - agents_list(b,2:3);   
                    POSb(b,:) = agents_list(b,2:3);
                end                                
                
            end

            if ~isempty(LOCb)

                % mean in the neighbors                       
                NNeighs = nnz(LOCb(:,1))+1;                
                muNeighs = sum(POSb)/NNeighs;                

                % difference in neihcbors
                DIFFb = sum(LOCb);

                % compute terms
                S2T1 = S2T1 + muNeighs.*(muNeighs - LOCa);                                            
                S2T2 = S2T2 + LOCa.*DIFFb/NNeighs;    

                % compute covariance
                S2tmp = S2tmp + (LOCa - muNeighs).^2;  
                SXYtmp = SXYtmp + prod(LOCa - muNeighs);                                   

            end
                        
        end        

        % variances terms        
        % S2 = (S2T1 + S2T2)/m;
        S2 = S2tmp/m;
        SX2 = S2(1);
        SY2 = S2(2);
        
        % symmetry term                   
        SXY = SXYtmp/m;        

        % correlation
        rho2 = SXY^2/(SX2*SY2);        
    else
        JS = 0;
        rho2 = rho2min;
    end  
    rho2conmin = (rho2min - rho2);

    % equality constraints
    ceq = [ceq; isrigid];
    ceq = [ceq; isConn];    

    % inequality constraints
    c = [c; distconmin];
    c = [c; distconmax];
    % c = [c; rho2conmin];
    % c = [c; InsideCircle];




end