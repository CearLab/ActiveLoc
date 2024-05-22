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

            % proceed only if neigbors are present
            if ~isempty(Neigh)
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

            % define pairs            
            pairs = nchoosek(pairsID,2);            
            % cycle over pairs            
            for i=1:size(pairs,1)                        
                rowTmp = find( (los_table(:,3) == pairs(i,1)) & (los_table(:,4) == pairs(i,2)) );
                pos = [pos; rowTmp];
            end   

            % get LOS
            obj.neigh.LOStab = los_table(unique(pos),:);

            else

                % al empty
                obj.neigh.ID = [];
                obj.neigh.D = [];
                obj.neigh.P = [];
                obj.neigh.Dmeas = [];
                obj.neigh.Pest = [];
                obj.neigh.LOStab = [];

            end

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
                [isRedundant, improveListRed] = checkRedundantRigidity(losTAB,agents_list);

                % init connectivity                
                [kconn, improveListCon] = checkKconnectivity(losTAB,agents_list,3);

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

                        % if something remains we can try removing
                        if ~isempty(losTABcopy)
                            tmp = find(agents_listcopy(:,1) == agentsrm);
                            agents_listcopy(tmp,:) = [];                        
        
                            % again redundancy, it should be by default, check
                            [isRedundant, ~] = checkRedundantRigidity(losTABcopy,agents_listcopy);                            
                            [kconn, ~] = checkKconnectivity(losTABcopy,agents_listcopy,3);
    
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

        % trilaterate from pos and dist
        function obj = trilaterate(obj,D,P)

            % This is quite strange, check 
            options = optimoptions('fminunc','Display','off');
            x0 = mean(P,1);
            Xhat = fminunc(@(x)cost_function_trilateration(x,D,P),x0,options);
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

            epsilon = manager.WS.epsilon;

            % first, if I have no neighbors, I randomly choose a direction
            if isempty(obj.neigh.ID) && (obj.agent_number ~= team.leader.agent_number)

                % move
                dstep = -epsilon*[1 1] + 2*epsilon*[1 1].*rand(1,2);
                obj.move(obj.location + dstep);

                % get neighjbors
                obj.getNeighbors;

                return; 
            end            

            % now we proceed checking your neighborhood characteristics:
            % first, check if neighbors are localized
            IDloc = [];
            for i=1:size(obj.neigh.ID,1)
                if agents{obj.neigh.ID(i)}.localized
                    IDloc = [IDloc, agents{obj.neigh.ID(i)}.agent_number];                    
                end
            end

            % if no neighbor is localized:           
            % I need to check the gradient of the distances and move
            % towards it. To do so I fo back and forth on the 4
            % cardinal points around my own position (I do this in FF),
            % so there is no need to know my location. Each time I
            % check the mean of my distances, which I measure again,
            % and in the end I decide where to move. Here we have
            % drift, but for small distances it's ok.
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
            % I do the same as in dgrad_search, but only with the distances
            % from localized neighors.
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

                    % I don't have, just stay still for the moment

                    % this will be passed to the optimization
                    return;

                else


                    % CASE 2

                    % If I have at least 3 localized neighbors, I can maximize 
                    % rigidity


                    % If we get here, it means that there is something to optimize.
                    % Proceed with the optimization. We do it the same way as the
                    % dgrad search. 
                    % rstep = drigid_search(obj,IDloc);
        
                    % move
                    % obj.move(obj.location + rstep);
        
                    % get neighjbors
                    % obj.getNeighbors;

                end

            end           

        end     

        % distance gradient research
        function dstep = dgrad_search(obj,IDloc)    

            manager = AgentManager.getInstance();

            % define 4 movements
            epsilon = manager.WS.epsilon;
            steps = epsilon*[   0 +1;     ...
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
            if isempty(IDloc)
                IDloc = N0; 
            end
            D0metric = mean(D0);
            

            % cycle over 4 steps
            for i=1:4          

                % move 
                obj.move(obj.location + steps(i,:));                
                obj.getNeighbors;     

                if ~isempty(obj.neigh.ID)

                    % find UWB distances
                    UWBpos = find(obj.neigh.Dmeas(:,2)==1); 
    
                    % get minimum distance
                    Dmin = min(obj.neigh.Dmeas(UWBpos,1));
    
                    % I need to check that no neighbors have been lost, at most
                    % gained.
                    UWBneigh = obj.neigh.ID(UWBpos);
                    
                    % you need to check if you lost ANY neighbor, not just
                    % localized ones. 
                    currentIdPos = [];
                    lostNeigh = zeros(1,numel(N0));
                    for j=1:numel(N0)
                        tmpPos = find(UWBneigh == N0(j));
                        if isempty(tmpPos)
                            lostNeigh(j) = 1;
                        else
                            lostNeigh(j) = 0;
                            try
                                locPos = find(UWBneigh == IDloc(j));
                                if ~isempty(locPos)
                                    currentIdPos = [currentIdPos locPos];
                                end
                            catch
                                % ok, the length of neighbors loc is less
                                % than all N0
                            end
                        end
                    end   

                else
                    lostNeigh = 1;
                end
    
                % thresh for distance
                DminThresh = manager.WS.DminThresh;

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

                % you need to check if you lost ANY neighbor, not just
                % localized ones. 
                currentIdPos = [];
                lostNeigh = ones(1,numel(N0));                

                while sum(lostNeigh) ~= 0                    

                    % move
                    dstep = -epsilon*[1 1] + 2*epsilon*[1 1].*rand(1,2);
                    obj.move(obj.location + dstep);
                    obj.getNeighbors;

                    if ~isempty(obj.neigh.ID)

                        % find UWB distances
                        UWBpos = find(obj.neigh.Dmeas(:,2)==1); 
        
                        % get minimum distance
                        Dmin = min(obj.neigh.Dmeas(UWBpos,1));
        
                        % I need to check that no neighbors have been lost, at most
                        % gained.
                        UWBneigh = obj.neigh.ID(UWBpos);
                        for j=1:numel(N0)
                            tmpPos = find(UWBneigh == N0(j));
                            if isempty(tmpPos)
                                lostNeigh(j) = 1;
                            else
                                lostNeigh(j) = 0;
                                currentIdPos = [currentIdPos tmpPos];
                            end
                        end
                    else
                        lostNeigh = 1;
                    end

                    if sum(lostNeigh) ~= 0
                        obj.move(obj.location - dstep);
                        obj.getNeighbors;
                    end
                end
            else
                [minD, pos] = min(Dsteps);            
                dstep = steps(pos,:);            
            end
            

        end

        % distance gradient research
        function dstep = drigid_search(obj,IDloc)                  

            % define 4 movements
            epsilon = manager.WS.epsilon;
            steps = epsilon*[   0 +1;     ...
                                0 -1;    ...
                                +1 0;     ...
                                -1 0    ];

            % get initial set of distances
            % find UWB and CAM distances
            UWBCAMpos = find(obj.neigh.Dmeas(:,2) > 0);                         
            N0 = unique(obj.neigh.ID(UWBCAMpos));

            % find IDpos
            IDpos = zeros(size(IDloc));            
            for i=1:numel(IDloc)
                IDpos(i) = find(obj.neigh.ID(:,1) == IDloc(i),1);
            end

            % now get LOStab UWB of only DistLoc (no nan in estpos)
            LOStmp = obj.neigh.LOStab( find( ~isnan(obj.neigh.LOStab(:,9)) & ~isnan(obj.neigh.LOStab(:,11)) & obj.neigh.LOStab(:,end) == 1), :);
            LOStmpMeA = obj.neigh.LOStab( find( isnan(obj.neigh.LOStab(:,9)) & obj.neigh.LOStab(:,3) == obj.agent_number & ~isnan(obj.neigh.LOStab(:,11)) ), :);
            LOStmpMeB = obj.neigh.LOStab( find( isnan(obj.neigh.LOStab(:,11)) & obj.neigh.LOStab(:,4) == obj.agent_number & ~isnan(obj.neigh.LOStab(:,9)) ), :);
            LOS = [LOStmp; LOStmpMeA; LOStmpMeB];
            agentList_tmp = [IDloc', obj.neigh.Pest(IDpos,1:2);];
            agentList_tmp(end+1,:) = [obj.agent_number, obj.location_est];

            % get rigidity matrix 
            R0 = calcRigitdyMatrix(LOS,agentList_tmp);
            R0over = R0'*R0;
            e0 = eig(R0over);    
            pos = find(abs(e0) < 1e-5);    
            isrigid0 = (numel(pos)==3);              
            e0(pos) = [];
            JR0 = min(e0);   

            % thresh for distance
            DminThresh = 2;
            

            % cycle over 4 steps
            for i=1:4          

                % move 
                obj.move(obj.location + steps(i,:));                
                obj.getNeighbors;    

                % find UWB distances
                UWBCAMpos = find(obj.neigh.Dmeas(:,2) > 0); 

                % get minimum distance
                Dmin = min(obj.neigh.Dmeas(UWBCAMpos,1));

                % I need to check that no neighbors have been lost, at most
                % gained.
                UWBneigh = unique(obj.neigh.ID(UWBCAMpos));

                % same but only with localized neighbors
                % UWBneigh = [];
                % for a=1:size(obj.neigh.ID,1)
                %     if ~isnan(obj.neigh.Pest(a,2))
                %         UWBneigh = [UWBneigh, obj.neigh.ID(a)];
                %     end
                % end

                % just to check what happens when i increase the neighbors
                if numel(UWBneigh) > numel(N0)
                    % ok nothing happens yay                    
                end
                
                currentIdPos = [];
                lostNeigh = zeros(1,numel(IDloc));
                for j=1:numel(IDloc)                       
                    tmpPos = find(UWBneigh == IDloc(j));
                    if isempty(tmpPos)
                        lostNeigh(j) = 1;
                    else
                        lostNeigh(j) = 0;
                        currentIdPos = [currentIdPos tmpPos];
                    end
                end                                

                % check conditions
                if (sum(Dmin < DminThresh)) || (nnz(lostNeigh)) 

                    % I've lost a neighbor, no good
                    isrigid(i) = 0;
                    JR(i) = nan;

                else

                    % now get LOStab UWB of only DistLoc (no nan in estpos)
                    LOStmp = obj.neigh.LOStab( find( ~isnan(obj.neigh.LOStab(:,9)) & ~isnan(obj.neigh.LOStab(:,11)) & obj.neigh.LOStab(:,end) == 1), :);
                    LOStmpMeA = obj.neigh.LOStab( find( isnan(obj.neigh.LOStab(:,9)) & obj.neigh.LOStab(:,3) == obj.agent_number & ~isnan(obj.neigh.LOStab(:,11)) ), :);
                    LOStmpMeB = obj.neigh.LOStab( find( isnan(obj.neigh.LOStab(:,11)) & obj.neigh.LOStab(:,4) == obj.agent_number & ~isnan(obj.neigh.LOStab(:,9)) ), :);
                    LOS = [LOStmp; LOStmpMeA; LOStmpMeB];
                    agentList_tmp = [IDloc', obj.neigh.Pest(IDpos,1:2);];
                    agentList_tmp(end+1,:) = [obj.agent_number, obj.location_est];
    
                    % get rigidity matrix 
                    R0 = calcRigitdyMatrix(LOS,agentList_tmp);
                    R0over = R0'*R0;
                    e = eig(R0over);    
                    pos = find(abs(e) < 1e-5);    
                    isrigid(i) = (numel(pos)==3);              
                    e(pos) = [];

                    if isrigid(i)
                        JR(i) = min(e);
                    else
                        JR(i) = nan;
                    end

                end

                % move back
                obj.move(obj.location - steps(i,:));                

            end

            % restore initial neighbors
            obj.getNeighbors;

            % find the best movement and go there (FF)
            if sum(isrigid) == 0 || sum(isnan(JR)) == numel(JR);
                dstep = [0 0];
            else
                [minJR, pos] = max(JR);            
                dstep = steps(pos,:);            
            end
            

        end         

    end

end