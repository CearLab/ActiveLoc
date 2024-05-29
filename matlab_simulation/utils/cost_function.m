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

    % is connected
    % A = calcAdjacencyMatrix(los_table,agents_list);
    % [allConn, A] = agentsOpt{1}.checkConnectivity(A);           

    % get agents_pos
    % agents_pos = agents_list(:,2:3);
    % sigma = std(agents_pos,1);
    % sigmacost = sigma*sigma';
    % JS = 1/(1+sigmacost);
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

        % just for fun
        Rover = R'*R;
        tr = trace(Rover);

        % set objectives
        JSA = 1/tr;
        JSB = rho2;        

        a = 0;
        Ta = manager.WS.Ta;
        Tb = manager.WS.Tb;
        JS = (a*Ta*JSA + (1-a)*Tb*JSB);  

        JS = JSB;

        % store
        manager.WS.JSA = JSA;
        manager.WS.JSB = JSB;
    else
        JS = 0;
    end        

    % Jtest
    Rover = R'*R;
    tr = trace(Rover);
    JT = -tr;
    

    % normalize
    % minterm = min(JD,JR);    
    J = 1*JR + 0*JS + 0*JT;

    if isempty(J)
        J = Inf;
    end

end