%% Agent class
% file: Agent.m
% author: Ido Sherf 
% date: 22/01/2024
% description: hndle class handling the creation of an agent on a Map 
% (see map class)
classdef Agent < handle

    % class properties
    properties

        % agent ID
        agent_number

        % position on the plane
        location

        % estimated position on the plane
        location_est

        % role
        role

        % team (maybe here I would return just the ID instead of all the team)
        team_id

        % neighborhood
        neigh

        % sensors
        sensors

        % localized
        localized

        % moved
        moved

    end

    % class methods
    methods

        % class constructor
        function obj = Agent(agent_number,location,team_id,role,location_est,neigh,sensors,localized,moved)

            % arguments validatio
            arguments
                agent_number
                location                
                team_id
                role = 'team_mate'
                location_est = [nan nan]
                neigh = []
                sensors = []
                localized = 0
                moved = 0
            end

            % assign values
            obj.agent_number = agent_number;
            obj.team_id = team_id;
            obj.location = location;
            obj.role = role;
            obj.location_est = location_est;
            obj.neigh = neigh;
            obj.sensors = sensors;
            obj.localized = localized;
            obj.moved = moved;

        end

        % get the neighborhood
        function obj = getNeighbors(obj)

            % get manager
            manager = AgentManager.getInstance();

            % get team
            team = manager.team_list{1};
            
            % get all agents            
            agents = {team.team_mates{1:end}};
            agents = {agents{1:team.leader.agent_number-1} team.leader agents{team.leader.agent_number:end}}; 

            % get agents summary tables
            [los_UWB,~] = calcLosMap(agents,'UWB');
            [los_CAM,~] = calcLosMap(agents,'CAM');
            los_table = [los_UWB; los_CAM];
            
            % get the neighborhood of agent
            sel_ID = obj.agent_number;
            % (CAM only)
            row01CAM = find( (los_table(:,3) == sel_ID) & (los_table(:,end) == 2) ); 
            row02CAM = find( (los_table(:,4) == sel_ID) & (los_table(:,end) == 2) );
            NeighCAM(:,1) = unique([los_table(row01CAM,4); los_table(row02CAM,3)]);
            NeighCAM(:,8) = 2;
            % (UWB only)
            row01UWB = find( (los_table(:,3) == sel_ID) & (los_table(:,end) == 1) ); 
            row02UWB = find( (los_table(:,4) == sel_ID) & (los_table(:,end) == 1) );
            NeighUWB(:,1) = unique([los_table(row01UWB,4); los_table(row02UWB,3)]);
            NeighUWB(:,8) = 1;            

            % wrap
            Neigh = [NeighUWB; NeighCAM];
            obj.neigh.NUWB = size(NeighUWB,1);
            obj.neigh.NCAM = size(NeighCAM,1);
            
            % before everything, get distance measures
            for i=1:size(NeighUWB,1)

                % true measure
                Neigh(i,2) = obj.sensors.UWB.getRangeMeas(Neigh(i,1));

                % noise measure
                Neigh(i,5) = obj.sensors.UWB.getRangeMeas(Neigh(i,1));

            end
            
            % get position of the neighbors
            for i=1:size(Neigh,1)
                Neigh(i,3:4) = agents{Neigh(i,1)}.location;
                Neigh(i,6:7) = agents{Neigh(i,1)}.location_est;
            end

            % set neighbors
            obj.neigh.ID = [Neigh(:,1) Neigh(:,8)];
            obj.neigh.D = [Neigh(:,2) Neigh(:,8)];            
            obj.neigh.P = [Neigh(:,3:4) Neigh(:,8)];
            obj.neigh.Dmeas = [Neigh(:,5) Neigh(:,8)];
            obj.neigh.Pest = [Neigh(:,6:7) Neigh(:,8)];

            % get the LOS of the neighbors       
            pos = unique([row01UWB; row02UWB; row01CAM; row02CAM;]);    % the los entries of the sel_ID

            % now get the others from newton combination of neigbors ID
            pairsID = unique([obj.neigh.ID(:,1); sel_ID]);
            pairs = nchoosek(pairsID,2);

            % cycle over pairs
            for i=1:size(pairs,1)
                rowTmp = find( (los_table(:,3) == pairs(i,1)) & (los_table(:,4) == pairs(i,2)) );
                pos = [pos; rowTmp];
            end

            % get LOS
            obj.neigh.LOStab = los_table(unique(pos),:);            

        end        

        % policy to get position
        function getPos(obj)                   

            % get manager
            manager = AgentManager.getInstance();

            % get team
            team = manager.team_list{1};
            
            % get all agents
            agents = {team.team_mates{1:end}};
            agents = {agents{1:team.leader.agent_number-1} team.leader agents{team.leader.agent_number:end}};            
            IDleader = team.leader.agent_number;            
            
            % init
            obj.neigh.improveList = obj.neigh.ID(:,1);
            
            % First case: if LOS with leader --> know position            
            if (obj.agent_number == IDleader)

                % here I know directly the position (should do with the bearing and stuff but for now...)
                obj.location_est = obj.location;
                obj.localized = 1;           
                obj.neigh.improveList = [];
    
                % find agents within range
                pos = find(obj.neigh.ID(:,2) == 2);

                for ag = 1:numel(pos)
                    neighagent = agents{obj.neigh.ID(pos(ag),1)};
                    neighagent.location_est = neighagent.location;
                    neighagent.localized = 1;
                    neighagent.neigh.improveList = [];
                end               

            else         

                % if you have the team leader in your CAM neighbors you are
                % localized (do not remove, because if you are in the CAM
                % but you have <3 IDloc, you set yourself as not localized)
                if find(obj.neigh.ID(:,1)==IDleader) & find(obj.neigh.ID(:,2)==2)
                    % here I know directly the position (should do with the bearing and stuff but for now...)
                    obj.location_est = obj.location;
                    obj.localized = 1;           
                    obj.neigh.improveList = [];
                    return;
                end

                % check if neighbors are localized
                IDloc = [];
                for i=1:size(obj.neigh.ID,1)
                    if agents{obj.neigh.ID(i)}.localized
                        IDloc = [IDloc, i];                    
                    end
                end

                % if no neighbor is localized, you can't localize yourself
                if numel(IDloc) < 3
                    obj.localized = 0;
                    obj.location_est = nan*[obj.location_est];
                    return;                    
                end

                % we need to find a globally rigid submesh before doing
                % trilateration                                              

                % get agent list only of those localized
                agents_list = [obj.neigh.ID(IDloc,1), obj.neigh.Pest(IDloc,:)]; 
                agents_list(end+1,:) = [obj.agent_number obj.location_est 0];

                % get LOS tab only for localized nodes and itself
                losTmp = obj.neigh.LOStab(find(obj.neigh.LOStab(:,end) == 1), :);
                rowLocStart = [];
                rowLocEnd = [];
                for j=1:size(agents_list,1)
                    rowLocStart = [rowLocStart; find(losTmp(:,3) == agents_list(j,1))]; 
                    rowLocEnd = [rowLocEnd; find(losTmp(:,4) == agents_list(j,1))];
                end
                rowLoc = intersect(rowLocStart,rowLocEnd);
                losTAB = losTmp(rowLoc,:);  

                % store this
                obj.neigh.LOC.losTab = losTAB;
                obj.neigh.LOC.agents_list = agents_list;

                % init redundancy
                [isRedundant, improveListRed] = checkRedundantRigidity(obj,losTAB,agents_list);

                % init connectivity                
                [kconn, improveListCon] = checkKconnectivity(obj,losTAB,agents_list,3);

                % remove itself
                improveListCon(find(improveListCon == obj.agent_number)) = [];

                % find the lines in losTAB with the subnet we are removing                                  
                if ~isempty(improveListCon)                           

                    % cycle over the candidate agents
                    for j=1:numel(improveListCon)

                        % start from initial LOS
                        losTABcopy = losTAB;
                        agents_listcopy = agents_list;

                        % select lines in LOS with this agent
                        improveListConLOS = find( ((losTABcopy(:,3) == improveListCon(j)) | (losTABcopy(:,4) == improveListCon(j))));                         

                        % merge improveLists
                        improveList = unique([improveListConLOS; improveListRed]);

                        % now if we're here, it is either not redundant or 
                        % 3-connected. let's try removing edges and check 
                        % again. I want to find the minimum redundant submesh                                

                        % find the agents to be removed 
                        losTABrm = losTABcopy(improveList,:);                                                
                        agentsrm = improveListCon(j);
    
                        % remove improved agents in lostab and agents_list
                        losTABcopy(improveList,:) = [];                        
                        tmp = find(agents_listcopy(:,1) == agentsrm);
                        agents_listcopy(tmp,:) = [];                        
    
                        % again redundancy, it should be by default, check
                        [isRedundant, ~] = checkRedundantRigidity(obj,losTABcopy,agents_listcopy);
                        [kconn, ~] = checkKconnectivity(obj,losTABcopy,agents_listcopy,3);

                        if (kconn && isRedundant)                            
                            for k=1:numel(IDloc)
                                if isempty(find(agents_listcopy(:,1) == obj.neigh.ID(IDloc(k),1)))
                                    IDloc(k) = -1;
                                end
                            end
                            pos = find(IDloc==-1);
                            IDloc(pos) = [];
                            break
                        end                            

                    end

                end                                
                

                % get location with trilateration 
                if kconn && isRedundant                    
                    D = obj.neigh.Dmeas(IDloc,1);
                    P = obj.neigh.Pest(IDloc,1:2);
                    obj.trilaterate(D,P);
                    obj.localized = 1;
                else           
                    obj.localized = 0;   
                    obj.location_est = nan*[obj.location_est];
                end
            end

        end

        % check if the neighborhood mesh is rigid
        function [isRigid, e] = isRigid(obj,losTAB,agents_list)

            % init
            isRigid = 0;

            % safety
            if isempty(losTAB)
                return;
            end
            
            % get rigidity matrix
            R = calcRigitdyMatrix(losTAB ,agents_list);

            % get nonzero eigs
            e = eig(R'*R);
        
            % get # nnz elements
            pos = find(abs(e) < 1e-10);

            e(pos) = [];
            isRigid = (numel(pos)==3);

        end

        % check redundant rigidity
        function [isRedundant, improveList] = checkRedundantRigidity(obj,losTAB,agents_list)
        
            % remove one edge at a time and check rigidity
            Nedges = size(losTAB,1);

            % init
            isRedundant = 0;
            fail = 0;
            improveList = [];

            % cycle over edges
            for i=1:Nedges

                % get LOS and check rigidity
                losTMP = losTAB;
                losTMP(i,:) = [];                
                isRigid = obj.isRigid(losTMP,agents_list);

                % if not rigid, we're alrready done
                if ~isRigid
                    fail = 1;
                    improveList(end+1,1) = i;                   
                end
            end

            % huray it's redundant
            if ~fail
                isRedundant = 1;
            end

        end

        % check if the neighborhood is 3-connected
        function [kconn, improveList] = checkKconnectivity(obj,losTAB,agents_list,k)

            % kconn flag
            kconn = 0;
            fail = 0;
            improveList = [];

            % if it does not have at least 2 vertices return
            if size(agents_list,1) < k
                return;
            end

            % start from the full neighbourhood graph. if its connected
            % then proceed, otherwise it's not k-connected for sure

            % call util
            A = calcAdjacencyMatrix(losTAB,agents_list);
            [allConn, A] = obj.checkConnectivity(A);

            if ~allConn
                return;
            else
                % now, we proceed by removing one k-tuple of vertices at a
                % time and check again connectivity

                % number of agents
                n = size(A,1);

                % pairs
                pairs = nchoosek(1:n,k-1);

                improveList = [];

                % cycle and check
                for i=1:size(pairs,1)
                    
                    % init
                    tmp = A;

                    % remove vertices
                    %removed one
                    tmpPair = pairs(i,:);
                    tmp(pairs(i,:),:) = [];
                    tmp(:,pairs(i,:)) = [];

                    isConn = obj.checkConnectivity(tmp);

                    % if you find a non-connected submesh then you drop the
                    % search
                    if (~isConn)
                        fail = 1;
                        improveList(end+1:end+numel(agents_list(tmpPair,1))) = agents_list(tmpPair,1);
                    end

                end

                % set the flag
                if ~fail 
                    kconn = 1;
                end
            end

        end

        % find submesh globally rigid
        function [isConn, Aout] = checkConnectivity(obj,A)

            % if scalar exit
            if isscalar(A)
                isConn = 1;
                return;
            end
            
            % get number nodes
            n = size(A,1);            

            % if A^(n-1) has no zero elements than is connected
            Apow = A^(n-1);
            Azeros = find(Apow == 0);
            if isempty(Azeros)
                isConn = 1;
            else
                isConn = 0;
            end

            % out
            Aout = A;

        end
        

        % trilaterate from pos and dist
        function obj = trilaterate(obj,D,P)

            % This is quite strange, check 
            x0 = mean(P,1);
            Xhat = fminunc(@(x)obj.cost_function_trilateration(x,D,P),x0);
            obj.location_est = Xhat;

        end

        % move in a new position
        function move(obj,p)            

            % set new location
            obj.location = p; 

            % reset localization
            obj.localized = 0;

            obj.moved = 1;

        end        

        % decide where to go
        function movePolicy(obj)            

            % not moved yet
            obj.moved = 0;            

            % get manager and team
            manager = AgentManager.getInstance();
            team = manager.team_list{1};            

            % get all agents
            agents = {team.team_mates{1:end}};
            agents = {agents{1:team.leader.agent_number-1} team.leader agents{team.leader.agent_number:end}};

            % thia function decides how to move in order to better localize
            % the agents, or to maximize coverage and rigidity. It is an
            % heuristic, and the goal depends on the conditions each agent
            % finds itself in. Thus, we first assess the agent conditions
            % and then we proceed with the motion policy. 

            % first we define the initial condition and the optimization
            % bounds, if present.

            % initial condition: here er directly see if we are localized
            % or not, as non localized agents have NaN as location_est
            x0 = obj.location_est;
            obj.neigh.x0 = x0;

            % first, if I have no neighbors, I randomly choose a direction
            if isempty(obj.neigh.ID) && (obj.agent_number ~= team.leader.agent_number)

                % move
                dstep = -0.5*[1 1] + [1 1].*rand(1,2);
                obj.move(obj.location + dstep);

                % get neighjbors
                obj.getNeighbors;

                return; 
            end

            % bounds (if loc_est = nan, just ignore bounds)
            % if it is not localized, no bounds, because they are defined
            % by the solvability of the localization problem. They are a
            % sort of passive bounds. 
            if obj.localized == 0
                LB = [];
                UB = [];
            else

                % if you are localized you can actually choose where to go,
                % without loosing your localization. So, we define a set of
                % active constraints, with the goal not to make you loose
                % your estimate.

                % First case: check if you have the leader within 
                % CAM distance. 
                pos = find((obj.neigh.ID(:,1) == team.leader.agent_number) & (obj.neigh.ID(:,2) == 2) );

                % if so, you don't want to lose CAM LOS with the leader,
                % beacuse you're directly localized. So, the upper bound
                % remains the estimated location
                if ~isempty(pos)
                    LB = obj.location_est - 0.5*team.leader.sensors.CAM.max_range;
                    UB = obj.location_est + 0.5*team.leader.sensors.CAM.max_range;
                % if not, the same bounds are applied but with UWB ranges
                else
                    LB = obj.location_est - 0.8*obj.sensors.UWB.max_range;
                    UB = obj.location_est + 0.8*obj.sensors.UWB.max_range;
                end
                
            end 

            % now we proceed checking your neighborhood characteristics:
            % first, check if neighbors are localized
            IDloc = [];
            for i=1:size(obj.neigh.ID,1)
                if agents{obj.neigh.ID(i)}.localized
                    IDloc = [IDloc, i];                    
                end
            end

            % if no neighbor is localized:            
            if isempty(IDloc) && (obj.agent_number ~= team.leader.agent_number)

                % find exploration step
                dstep = obj.dgrad_search(IDloc);

                % move
                obj.move(obj.location + dstep);

                % get neighjbors
                obj.getNeighbors;

                return; 
            end

            % if you have someone localized, we can work on something.            
            % Now let's see what we can do
            % role: 
            %       1: not localized
            %       2: localized

            % what happens when I am not localized:
            % I need to check the gradient of the distances and move
            % towards it. To do so I fo back and forth on the 4
            % cardinal points around my own position (I do this in FF),
            % so there is no need to know my location. Each time I
            % check the mean of my distances, which I measure again,
            % and in the end I decide where to move. Here we have
            % drift, but for small distances it's ok.
            if obj.localized == 0                     

                % find exploration step
                dstep = obj.dgrad_search(IDloc);

                % move
                obj.move(obj.location + dstep);

                % get neighjbors
                obj.getNeighbors;

                return;                    

            % what happens when I am localized:
            % more cases, I have 
            else

                % if I am the leader, just stay still
                if obj.agent_number == team.leader.agent_number
                    return;
                end
                
                % I try to maximize rigidity, but I need at least 2
                % neighbors to compute rigidity

                if numel(IDloc) < 3

                    % CASE 1

                    % If I don't have 2 localized neighbors, just move away
                    % from those that I have. In future this could be 
                    % maximizing coverage

                    % this will be passed to the optimization
                    CASE = 1;

                else


                    % CASE 2

                    % If I have at least 3 localized neighbors, I can maximize 
                    % rigidity

                    % this will be passed to the optimization
                    CASE = 2;

                end

            end
            
            % If we get here, it measn that there is something to optimize.
            % Proceed with the optimization.                       

            % optimization settings 
            options = optimoptions('fmincon','Display','iter','Algorithm','sqp');


            [pdes, J] = fmincon(                     ...
                                @(x)obj.cost_function_policy(x,CASE,IDloc), ...
                                x0,                     ...
                                [],                     ...
                                [],                     ...
                                [],                     ...
                                [],                     ...
                                LB,                     ...
                                UB,                     ...
                                [],                     ...
                                options);              

            % if they tell to stay where we first where, do so
            if (sum(pdes ~= x0) == 0) || isnan(sum(pdes))
                return
            else                                

                % control
                % distance from estimated position
                dist = pdes - obj.location;

                % proportional
                k = 0.1;

                posstep = obj.location + k*dist;  

                % move
                obj.move(posstep);

            end

        end     

        % distance gradient research
        function dstep = dgrad_search(obj,DistLoc)                  

            % define 4 movements
            steps = 0.5*[   0 +1;     ...
                            0 -1;    ...
                            +1 0;     ...
                            -1 0    ];

            % distances storage
            Dsteps = zeros(1,4);

            % get initial set of distances
            % find UWB distances
            UWBpos = find(obj.neigh.Dmeas(:,2)==1);             
            D0 = obj.neigh.Dmeas(UWBpos,1);
            N0 = obj.neigh.ID(UWBpos);

            % handle no localized neighbors
            if isempty(DistLoc)
                DistLoc = 1:numel(UWBpos);
            end
            D0metric = mean(D0(DistLoc));
            

            % cycle over 4 steps
            for i=1:4          

                % move 
                obj.move(obj.location + steps(i,:));                
                obj.getNeighbors;                     

                % find UWB distances
                UWBpos = find(obj.neigh.Dmeas(:,2)==1); 

                % get minimum distance
                Dmin = min(obj.neigh.Dmeas(UWBpos,1));

                % I need to check that no neighbors have been lost, at most
                % gained.
                UWBneigh = obj.neigh.ID(UWBpos);

                % just to check what happens when i increase the neighbors
                if numel(UWBneigh) > numel(N0)
                    % ok nothing happens yay
                end
                
                currentIdPos = [];
                lostNeigh = zeros(1,numel(DistLoc));
                for j=1:numel(DistLoc)
                    tmpPos = find(UWBneigh == N0(DistLoc(j)));
                    if isempty(tmpPos)
                        lostNeigh(j) = 1;
                    else
                        lostNeigh(j) = 0;
                        currentIdPos = [currentIdPos tmpPos];
                    end
                end                

                % thresh for distance
                DminThresh = 2;

                % check conditions
                if (nnz(lostNeigh)) || (sum(Dmin < DminThresh))

                    % I've lost a neighbor, no good
                    Dsteps(i) = Inf;

                else

                    % get the current UWB meas
                    D = obj.neigh.Dmeas(currentIdPos,1);                                                
    
                    % find mean
                    Dsteps(i) = mean(D);

                end

                % move back
                obj.move(obj.location - steps(i,:));                

            end

            % restore initial neighbors
            obj.getNeighbors;

            % find the best movement and go there (FF)
            if sum(isinf(Dsteps)) == numel(Dsteps)
                dstep = [0 0];
            else
                [minD, pos] = min(Dsteps);            
                dstep = steps(pos,:);            
            end
            

        end

        % cost function for trilateration
        function J = cost_function_trilateration(obj,X,D,P)            

            % n meas
            nmeas = numel(D);

            % get cost function
            Pdiff = P - X;

            % init
            J = 0;

            % build diff
            for i=1:nmeas
                J = J + (Pdiff(i,:)*Pdiff(i,:)' - D(i)^2)^2;
            end            

        end

        % cost function
        function J = cost_function_policy(obj,x,CASE,IDloc)

            % get manager and team
            manager = AgentManager.getInstance();                       

            % get plane or space info
            p = manager.WS.p;

            % CASE 1
            % I have neighbors but less than two of them are localized. 
            % I just move away from them.
            if CASE == 1

                % stay here
                J = 0;
                return;

                % get agent list only of those localized
                agents_list = [obj.neigh.ID(IDloc,1), obj.neigh.Pest(IDloc,:)]; 
    
                % define pdes as the barycenter of them
                G = mean(agents_list(find(agents_list(:,end) == 1),2:3),1);                    
    
                % compute your distance from the barycenter
                delta = G - x;
                dbar = norm(delta);
    
                % compute bearing from initial condition                    
                currbear = atan2(delta(2),delta(1));                    
                bear = abs(currbear - pi*sign(currbear));
    
                % maximize this distance
                J = 0/dbar + 1*bear;

            % CASE 2
            % I have at least three neighbors localized. 
            % I want to move to maximize rigidity.
            elseif CASE == 2

                % stay here
                J =0;
                return;

                % get agent list only of those localized
                agents_list = [obj.neigh.ID(IDloc,1), obj.neigh.Pest(IDloc,:)]; 
                agents_list(end+1,:) = [obj.agent_number x 0];
        
                % get LOS tab only for localized nodes and itself
                losTmp = obj.neigh.LOStab(find(obj.neigh.LOStab(:,end) == 1), :);
                rowLocStart = [];
                rowLocEnd = [];
                for j=1:size(agents_list,1)
                    rowLocStart = [rowLocStart; find(losTmp(:,3) == agents_list(j,1))]; 
                    rowLocEnd = [rowLocEnd; find(losTmp(:,4) == agents_list(j,1))];
                end
                rowLoc = intersect(rowLocStart,rowLocEnd);
                losTAB = losTmp(rowLoc,:);  

                % replace the estimated location with the actual x
                for i=1:size(losTAB,1)
                    % if agent is the first in the row pair
                    if losTAB(i,3) == obj.agent_number
                        losTAB(i,5:5+p-1) = x;
                        losTAB(i,5+2*p:5+3*p-1) = x;
                    end
                    % if agent is the second in the row pair
                    if losTAB(i,4) == obj.agent_number
                        losTAB(i,5+p:5+2*p-1) = x;
                        losTAB(i,5+3*p:5+4*p-1) = x;
                    end
                end
                
                % check theorem
                [kconn, improveListConn] = checkKconnectivity(obj,losTAB,agents_list,3);
                [isRedundant, improveListRed] = checkRedundantRigidity(obj,losTAB,agents_list);                
                    
                % if rigid, get lambda4 and maximize
                if kconn && isRedundant         
                    [isRigid, e] = obj.isRigid(losTAB,agents_list);
                    J = 1/min(e);                                    
                % if not globally rigid not good
                else                        
                    J = 1e5;                                        
                end                

            end                                                  

        end

    end

end