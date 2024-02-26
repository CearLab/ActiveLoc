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
% agents_pos = [  0  0; -3  -3; 0  3;   +3  -3];
% agents_pos = [  -1.3276   -1.6517; -7.9982   -1.2929;   -5.6519   -4.7288;  -5.0198   -7.5618];
agents_pos = [  -1.3276   -1.6517;  ...     -1.3276   -1.6517;  ...
                 3.5252    0.6211;  ...     +3.5252    0.6211;   ...
                -7.9982   -1.2929;  ...     -7.9982   -1.2929;  ...
                -3.1627    2.9635;  ...     -3.1627    2.9635;  ...
                -5.6519   -4.7288;  ...     -5.6519   -4.7288;  ...
                -1.5226    7.8000;  ...     -6.5226    6.0499;  ...
                -5.0198   -7.5618;  ...     -5.0198   -7.5618;  ...       
                -0.4710    0.7275   ];

% number of agents
m = size(agents_pos,1);

% create agents of team 1
% random pick of the agent position
% agents_pos = rand(m,2)*16 - 8;   

% define leader
leaderID = 3;

% translate everything to have the leader in the origin
dP = agents_pos(leaderID,:);
agents_pos = agents_pos - dP;

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
        agents{i}.sensors.CAM = Sensor(agents{i}.agent_number,'range',5);
        agents{i}.sensors.UWB = Sensor(agents{i}.agent_number,'range',8);
    else
        agents{i}.sensors.UWB = Sensor(agents{i}.agent_number,'range',8);
    end
end 


%% LOCALIZATION for agents
if 1    
    
    % init
    notLocalized = 1;

    % plot
    f1 = plotFleet(f1,gifFile,flag,delay,1,agents_pos);
    
    % localization policy
    while notLocalized
    
        % two steps of localization
        for i = 1:m                
    
            % localize
            agent = agents{i};
            agent.get_neighbors;
            agent.getPos;
            agent.movePolicy;

            % plot                
            f1 = plotFleet(f1,gifFile,flag,delay,5,agents_pos);

        end                            
    
        % plot
        f1 = plotFleet(f1,gifFile,flag,delay,1,agents_pos);
    
        % check all network localization
        loc = zeros(1,m);
        for i=1:m
            agent = agents{i};
            loc(i) = agent.localized;
        end    
    
        % if all localized exit otherwise not
        if (prod(loc) == 1)
            notLocalized = 0;
        end
    
    end
    
end

%% plot section
f1 = plotFleet(f1,gifFile,flag,delay,1,agents_pos);
% h5 = drawLosMap(agents{2}.neigh.LOStab,f2,4,2);