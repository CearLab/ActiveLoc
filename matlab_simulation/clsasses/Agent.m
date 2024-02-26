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

    end

    % class methods
    methods

        % class constructor
        function obj = Agent(agent_number,location,team_id,role,location_est,neigh,sensors,localized)

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

        end

        % get the neighborhood
        function obj = get_neighbors(obj)

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

            % check if submesh is rigid (also contains itself)        
            agents_list = [obj.neigh.ID(1:obj.neigh.NUWB,1), obj.neigh.P(1:obj.neigh.NUWB,1:2)];            
            agents_list(end+1,:) = [obj.agent_number obj.location];
            [~, pos] = sort(agents_list(:,1));
            agents_list = agents_list(pos,:);
            obj.neigh.isRigid = obj.isRigid(obj.neigh.LOStab,agents_list);

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

            % if locaslized skip
            if (obj.localized == 1) && (obj.agent_number ~= IDleader)
                obj.neigh.improveList = [];
                return;
            end
            

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

                % check if neighbors are localized
                IDloc = [];
                for i=1:size(obj.neigh.ID,1)
                    if agents{obj.neigh.ID(i)}.localized
                        IDloc = [IDloc, i];                    
                    end
                end

                % if no neighbor is localized, you can't localize yourself
                if numel(IDloc) < 3
                    return;                    
                end

                % we need to find a globally rigid submesh before doing
                % trilateration                                              

                % get agent list only of those localized
                agents_list = [obj.neigh.ID(IDloc,1), obj.neigh.P(IDloc,:)]; 
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

                % check theorem
                kconn = checkKconnectivity(obj,losTAB,agents_list,3);
                [isRedundant, improveList] = checkRedundantRigidity(obj,losTAB,agents_list);

                % save improve directions
                obj.neigh.improveList = improveList;

                % get location with trilateration 
                if kconn && isRedundant
                    obj.neigh.GlobalRigid = 1;
                    D = obj.neigh.Dmeas(IDloc,1);
                    P = obj.neigh.Pest(IDloc,1:2);
                    obj.trilaterate(D,P);
                    obj.localized = 1;
                else                    
                    obj.neigh.GlobalRigid = 0;
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
            isRigid = (numel(pos)==3);

        end

        % check redundant rigidity
        function [isRedundant, improveList] = checkRedundantRigidity(obj,losTAB,agents_list)
        
            % remove one edge at a time and check rigidity
            Nedges = size(losTAB,1);

            % init
            isRedundant = 0;
            improveList = [];

            % cycle over edges
            for i=1:Nedges

                % get LOS and check rigidity
                losTMP = losTAB;
                losTMP(i,:) = [];                
                isRigid = obj.isRigid(losTMP,agents_list);

                % if not rigid, we're alrready done
                if ~isRigid
                    improveList = [improveList, i];
                    return
                end
            end

            % huray it's redundant
            isRedundant = 1;

        end

        % check if the neighborhood is 3-connected
        function kconn = checkKconnectivity(obj,losTAB,agents_list,k)

            % kconn flag
            kconn = 0;

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
                pairs = nchoosek(1:n,k);

                % cycle and check
                for i=1:size(pairs,1)
                    
                    % init
                    tmp = A;

                    % remove vertices
                    tmp(pairs(i,:),:) = [];
                    tmp(:,pairs(i,:)) = [];

                    isConn = obj.checkConnectivity(tmp);

                    % if you find a non-connected submesh then you drop the
                    % search
                    if (~isConn)
                        return;
                    end

                end

                % set the flag
                kconn = 1;
            end

        end

        % find submesh globally rigid
        function [isConn, Aout] = checkConnectivity(obj,A)

            % if scalar exit
            if isscalar(A)
                isConn = 1;
                return;
            end

            % remove empty rows (no connections)
            row = [];
            for i=1:size(A,1)
                if (sum(A(i,:) == 1) == 0)
                    row = [row i];
                end
            end

            % remove useless rows
            A(row,:) = [];
            A(:,row) = [];

            % get number nodes
            n = size(A,1);

            % if A^(n-1) has no zero elements than is connected
            isConn = (nnz(A^(n-1)) ~= 0);

            % out
            Aout = A;

        end
        

        % trilaterate from pos and dist
        function obj = trilaterate(obj,D,P)

            % This is quite strange, check 
            x0 = mean(P,1);
            Xhat = fminunc(@(x)obj.cost_function(x,D,P),x0);
            obj.location_est = Xhat;

        end

        % cost function for trilateration
        function J = cost_function(obj,X,D,P)            

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

        % decide where to go
        function movePolicy(obj)

            if  ~obj.localized                

                % currently try to reconnect neighborhood
                % remember that you also have obj.neigh.improveList

                % leader position - is the origin!
                % manager = AgentManager.getInstance();
                % team = manager.team_list{1};
                % posLeader = team.leader.location;
                posLeader = [0, 0];

                % distance from leader
                dist = posLeader - obj.location;

                % proportional
                k = 0.1;

                pdes = obj.location + k*dist;            
                obj.move(pdes);

            else

                % try to maximize the rigidity of the network 
                % localized around you

                % store this
                % obj.neigh.LOC.losTab;
                % obj.neigh.LOC.agents_list;
                % 
                % % optimization
                % options = optimoptions( 'patternsearch', ...
                %                         'MeshTolerance',1e-6, ...
                %                         'InitialMeshSize',1e2, ...
                %                         'UseParallel', true, ...
                %                         'MaxFunctionEvaluations', 1e4, ...
                %                         'ConstraintTolerance', 1e-4, ...
                %                         'StepTolerance', 1e-10);
                % 
                % % init counter
                % tic
                % disp('Optimizing')
                % [X, J] = patternsearch(                     ...
                %                     @(x)cost_function(x,p), ...
                %                     X0,                     ...
                %                     [],                     ...
                %                     [],                     ...
                %                     [],                     ...
                %                     [],                     ...
                %                     Inf*LB,                   ...
                %                     Inf*UB,                   ...
                %                     @(x)nonlcon(x,p));
                % 
                % [isRigid, e] = isRigid(obj,losTAB,agents_list);
            end           

        end

        % move in a new position
        function move(obj,p)            

            % set new location
            obj.location = p;

            % reset localized
            obj.localized = 0;

        end

        % cost function
        function J = cost_function_policy(obj,x,p)

            J = 1/min(e);
            % J = max(e)/min(e);
        
            if ~isscalar(J)
                J = Inf;
            end

        end

    end

end