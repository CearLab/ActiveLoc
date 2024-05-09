%% Sandbox
% file: sanbox.m
% author: Federico Oliva 
% date: 19/02/2024
% description: test agents_posalization on neighborhood

%% test section
% clc;clear;close all;
% rng(2)

% plot setup
delay = 1e-3;
flag = 0;

% delete gifs
if flag
    delete('SimAnimatedF1.gif')
    gifFile = 'SimAnimatedF1.gif';
    f1 = figure(1);
else
    f1 = [];
    gifFile = [];
end

%%% define map  .
map = Map.getInstance();

%%% create agent manager
manager = AgentManager.getInstance();
manager.reset();

% create agents of team 1
% define initial condition
agents_pos = zeros(8,2);

% define pos
% agents_pos = [  -1.3276   -1.6517;  ...     -1.3276   -1.6517;  ...
%                  3.5252    0.6211;  ...     +3.5252    0.6211;   ...
%                 -7.9982   -1.2929;  ...     -7.9982   -1.2929;  ...
%                 -3.1627    2.9635;  ...     -3.1627    2.9635;  ...
%                 -5.6519   -4.7288;  ...     -5.6519   -4.7288;  ...
%                 -1.5226    7.8000;  ...     -6.5226    6.0499;  ...
%                 -5.0198   -7.5618;  ...     -5.0198   -7.5618;  ...       
%                 -0.4710    0.7275   ];

% not working
agents_pos = [  -10.8000  -10.8000; ... 1.8459    7.9805; ...0.6109   -3.1477; ...7.1731    2.6704;
                -10.8000    2.4745; ...-9.1801    5.8834; ...8.8026   -0.9216; ...-1.8811   -9.9225;  
                 0         0; ...0         0; ...0         0; ... 0         0; ...    
                -7.6534    5.3542; ...8.7676    4.4313; ...1.3499   -7.3757; ...-4.7660    2.1460; ... 
                -10.5148   -6.8540; ...0.8094   -6.2154; ...1.8160   -3.1430; ...5.7518    2.8844; ...
                -10.8000   -3.6700; ...-4.7966    1.1352; ...1.2413  -10.8000; ...2.7571  -10.2139; ...
                -3.0213    2.6201; ...3.6332    0.3495; ...-5.0193    1.0384; ...-4.9516   -5.7807; ...
                -10.1525    4.4831]; ...-5.7516    7.2725]; ...10.8000   -5.1919]; ...10.5794   -9.5845]; 

% number of agents
m = size(agents_pos,1);

% dimension
p = size(agents_pos,2);
manager.WS.p = p;

% define leader
leaderID = 3;

% sensor setup
CAMrange = 3;
UWBrange = 8;

% generate until connection
allConn = 0;
Dmin = 0;
Dminthresh = 2;
DminthreshInit = 1;
epsilon = 0.75;
while (allConn==0) || (Dmin < DminthreshInit)

    if (allConn==0) || (Dmin < DminthreshInit)
        manager.reset();
        manager.WS.p = p;
        manager.WS.DminThresh = Dminthresh;
        manager.WS.epsilon = epsilon;
    end

    % random 
    for i=1:p
        a = 0.9*map.map_span(i,1);
        b = 0.9*map.map_span(i,2);
        agents_pos(:,i) = a + (b-a).*rand(m,1);  % random pick of the agent position    
    end

    % translate everything to have the leader in the origin
    dP = agents_pos(leaderID,:);
    agents_pos = agents_pos - dP;

    % max over the map span
    agents_pos(:,1) = max(min(agents_pos(:,1),0.9*map.map_span(1,2)),0.9*map.map_span(1,1));
    agents_pos(:,2) = max(min(agents_pos(:,2),0.9*map.map_span(2,2)),0.9*map.map_span(2,1));
    
    % create agents
    for i = 1:m
        if i == leaderID
            manager.createAgent(agents_pos(i,:),1,'team_leader');         
        else
            manager.createAgent(agents_pos(i,:),1,'team_mate');         
        end
    end 
    
    % get all agents
    agents = manager.getAllAgent();
    
    % set sensors
    for i = 1:m
        if i == leaderID
            agents{i}.sensors.CAM = Sensor(agents{i}.agent_number,'range',CAMrange);
            agents{i}.sensors.UWB = Sensor(agents{i}.agent_number,'range',UWBrange);
        else
            agents{i}.sensors.UWB = Sensor(agents{i}.agent_number,'range',UWBrange);
        end
    end 

    % get the team you just created
    team = manager.team_list{1};

    % get all agents from the tea6m 
    agents = {team.team_mates{1:end}};
    agents = {agents{1:team.leader.agent_number-1} team.leader agents{team.leader.agent_number:end}};    

    % LOS calculations
    [los_table,agents_list] = calcLosMap(agents,'UWB');        
    
    if ~isempty(los_table)              
        % get distances
        D = los_table(:,5:5+p-1) - los_table(:,5+p:5+2*p-1);
        % I also want to ensure a min distance between agents
        Dmin = min(vecnorm(D,2,2));        
        % check connectivity        
        A = calcAdjacencyMatrix(los_table,agents_list);
        [allConn, A] = checkConnectivity(A); 
    else
        Dmin = 0;
        allConn = 0;
    end

end

%% LOCALIZATION for agents
if 1    
    
    % init
    notLocalized = 1;

    % init iter
    iter = 0;
    MaxIter = 200;

    % plot
    f1 = plotFleet(f1,gifFile,flag,delay,1,agents_pos); 
    
    % localization policy
    while (iter < MaxIter) && (notLocalized)

        % iter increment
        iter = iter + 1;

        % clc
        % disp(['iteration: ', num2str(iter)]);

        % sequence        
        seq = randperm(m);
    
        % two steps of localization
        for i = 1:m                
    
            % localize
            agent = agents{seq(i)};
            agent.getNeighbors;
            agent.getPos;  

            % plot                
            % f1 = plotFleet(f1,gifFile,flag,delay,5,agents_pos);

            % try to move            
            agent.movePolicy;
            % agent.getPos;

        end

        % plot                
        if mod(iter,2) == 1
            f1 = plotFleet(f1,gifFile,flag,delay,5,agents_pos);
        end

        
    
        % check all network localization
        loc = zeros(1,m);
        for i=1:m
            agent = agents{i};
            loc(i) = agent.localized;            
        end    
    
        manager.WS.locstory(:,iter) = loc;
        % if all localized exit otherwise not
        if (prod(loc) == 1)

            notLocalized = 0;                  

        end
    
    end
    
end

%% result analysis
for i=1:m
    agentsEst(i,:) = [agents{i}.location agents{i}.location_est];
end
agentsEstDiff = agentsEst(:,1:2)-agentsEst(:,3:4);

%% plot section
f1 = plotFleet(f1,gifFile,flag,delay,1,agents_pos);