%% Sandbox
% file: sanbox.m
% author: Federico Oliva 
% date: 19/02/2024
% description: test agents_posalization on neighborhood

%% test section
clc;clear;close all;
rng(1)

% plot setup
delay = 1e-3;
flag = 1;

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

% define pos
agents_pos = [  -1.3276   -1.6517;  ...     -1.3276   -1.6517;  ...
                 3.5252    0.6211;  ...     +3.5252    0.6211;   ...
                -7.9982   -1.2929;  ...     -7.9982   -1.2929;  ...
                -3.1627    2.9635;  ...     -3.1627    2.9635;  ...
                -5.6519   -4.7288;  ...     -5.6519   -4.7288;  ...
                -1.5226    7.8000;  ...     -6.5226    6.0499;  ...
                -5.0198   -7.5618;  ...     -5.0198   -7.5618;  ...       
                -0.4710    0.7275   ];

% create agents of team 1
% define initial condition
% agents_pos = zeros(15,2);
% number of agents
m = size(agents_pos,1);

% dimension
p = size(agents_pos,2);
manager.WS.p = p;

% define leader
leaderID = 3;

% % random 
% for i=1:p
%     a = 0.9*map.map_span(i,1);
%     b = 0.9*map.map_span(i,2);
%     agents_pos(:,i) = a + (b-a).*rand(m,1);  % random pick of the agent position    
% end

% translate everything to have the leader in the origin
dP = agents_pos(leaderID,:);
agents_pos = agents_pos - dP;

% sensor setup
CAMrange = 3;
UWBrange = 8;

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


%% LOCALIZATION for agents
if 1    
    
    % init
    notLocalized = 1;

    % init iter
    iter = 0;
    MaxIter = 100;

    % plot
    f1 = plotFleet(f1,gifFile,flag,delay,1,agents_pos); 
    
    % localization policy
    while (notLocalized) && (iter < MaxIter)

        % iter increment
        iter = iter + 1;

        clc
        disp(['iteration: ', num2str(iter)]);

        % sequence        
        seq = randperm(m);
    
        % two steps of localization
        for i = 1:m                
    
            % localize
            agent = agents{seq(i)};
            agent.getNeighbors;
            agent.getPos;  

            % plot                
            f1 = plotFleet(f1,gifFile,flag,delay,5,agents_pos);

            % try to move            
            agent.movePolicy;                             

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
% h5 = drawLosMap(agents{2}.neigh.LOStab,f2,4,2);