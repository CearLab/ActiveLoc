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

        % batch_len
        batchLen
        batchCount
        agent_counter
        agents_added

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

            obj.batchLen = 0;
            obj.batchCount = 0;
            obj.agent_counter = 0;
            obj.agents_added = [];

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
            agent_numbers = [];
            for i=1:length(agents)
                agent_numbers = [agent_numbers agents{i}.agent_number];
            end
            for i=1:size(Neigh,1)
                pos = find(agent_numbers == Neigh(i,1));
                Neigh(i,3:4) = agents{pos}.location;
                Neigh(i,6:7) = agents{pos}.location_est;
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

                agent_numbers = [];
                for i=1:length(agents)
                    agent_numbers = [agent_numbers agents{i}.agent_number];
                end

                for ag = 1:numel(pos)
                    pos_agent = find(agent_numbers == obj.neigh.ID(pos(ag)));
                    neighagent = agents{pos_agent};
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

                agent_numbers = [];
                for i=1:length(agents)
                    agent_numbers = [agent_numbers agents{i}.agent_number];
                end

                % check if neighbors are localized
                IDloc = [];
                for i=1:size(obj.neigh.ID,1)
                    pos_agent = find(agent_numbers == obj.neigh.ID(i,1));
                    if agents{pos_agent}.localized
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
                    try
                    obj.trilaterate(D,P);
                    catch
                    obj.trilaterate(D,P);
                    end
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

            % initial condition: here er directly see if we are localized
            % or not, as non localized agents have NaN as location_est
            x0 = obj.location_est;
            obj.neigh.x0 = x0;

            epsilon = manager.WS.epsilon;

            % first, if I have no neighbors I stay still
            if isempty(obj.neigh.ID) && (obj.agent_number ~= team.leader.agent_number)                
                return; 
            end            

            % now we proceed checking your neighborhood characteristics:
            % first, check if neighbors are localized
            agent_numbers = [];
            for i=1:length(agents)
                agent_numbers = [agent_numbers agents{i}.agent_number];
            end

            IDloc = [];
            for i=1:size(obj.neigh.ID,1)
                pos_agent = find(agent_numbers == obj.neigh.ID(i,1));               
                IDloc = [IDloc, agents{pos_agent}.agent_number];                                
            end
            IDloc = unique(IDloc);

            % if no neighbor is localized:                       
            % if isempty(IDloc) && (obj.agent_number ~= team.leader.agent_number)   
                % obj.policy_wrapper(IDloc);                
                % return; 
            % end

            % if you have some neighbor localized            

            % what happens when I am not localized: I stay still            
            if obj.localized == 0  
                
                obj.policy_wrapper(IDloc);
                return;                    

            % what happens when I am localized:           
            else                

                % obj.policy_wrapper(IDloc);
                % return;                    

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
                        if N0(j) <= obj.agent_counter
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
                        else
                            % I'm not using batch traces as lost neighbors
                            a = 1;
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
                dstep = 0.*steps(1,:);
            else
                [minD, pos] = min(Dsteps);            
                dstep = steps(pos,:);            
            end
            

        end
        
        % wrapper for the policy
        function policy_wrapper(obj,IDloc)            

            % find exploration step                                                
            dstep = obj.dgrad_search(IDloc);                

            % random movement
            % dstep = -epsilon*[1 1] + 2*epsilon*[1 1].*rand(1,2);               

            % move
            oldposition = obj.location;
            obj.move(obj.location + dstep);            

            if sum(abs(dstep)) ~= 0
                obj.batch_store(oldposition);
            end

            % get neighjbors            
            obj.getNeighbors;
          
        end

        % handle the batch
        function batch_store(obj,position)

            % get team
            manager = AgentManager.getInstance();            
            team = manager.team_list{1};
            agents = {team.team_mates{1:end}};
            agents = {agents{1:team.leader.agent_number-1} team.leader agents{team.leader.agent_number:end}}; 

            % handle batch
            if obj.batchCount < obj.batchLen

                % check if there is another agent with same pose
                for i=1:length(agents)
                    if (agents{i}.location == position) & (agents{i}.agent_number ~= obj.agent_number)
                        return
                    end
                end

                % create agent
                manager.createAgent(position,1,'team_mate');               
    
                % get all agents
                agents = {team.team_mates{1:end}};
                agents = {agents{1:team.leader.agent_number-1} team.leader agents{team.leader.agent_number:end}}; 
                agent = agents{end};            
                agent.sensors.UWB = Sensor(agent.agent_number,'range',obj.sensors.UWB.max_range);
                agent.location_est = obj.location_est;
                agent.localized = obj.localized;
                obj.agents_added = [obj.agents_added, agent.agent_number];

                obj.batchCount = obj.batchCount + 1;
            else
                pos = [];
                % team mates
                for i = 1:length(team.team_mates)
                    if ~isempty(obj.agents_added) && team.team_mates{i}.agent_number == obj.agents_added(1)
                        pos = [pos i];
                        obj.agents_added(1) = [];
                        manager.agents_counter = manager.agents_counter - 1;
                    end                    
                end                                
                team.team_mates(pos) = [];
                obj.batchCount = 0;
            end           

            % sort agents            
            scaled = zeros(1,numel(team.team_mates));
            agents_id = [];
            while prod(scaled) == 0                

                scaled = zeros(1,numel(team.team_mates));
                for i=1:numel(team.team_mates)            
                    if team.team_mates{i}.agent_number > manager.agents_counter
                        team.team_mates{i}.agent_number = team.team_mates{i}.agent_number - 1;

                        for j=1:numel(team.team_mates)
                            agents_id = [agents_id team.team_mates{j}.agent_number];
                        end
                        agents_id = [agents_id team.leader.agent_number];

                        if numel(find(team.team_mates{i}.agent_number == agents_id)) > 1
                            team.team_mates{i}.agent_number = team.team_mates{i}.agent_number - 1;
                        end
                        scaled(i) = 0;
                    else
                        scaled(i) = 1;
                    end
                end            
            end            

        end
        
    end

end