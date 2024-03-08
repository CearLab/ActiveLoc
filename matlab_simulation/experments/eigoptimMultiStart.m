%% Eignevalues optimization
% file: eigoptim.m
% author: Federico Oliva 
% date: 01/02/2024
% description: test on lambda4 interpretation

%% init 
clc;
clear;
close all;
rng(1);

%% check toolbox dependencies 
tb = ver; assert(any(strcmp('Optimization Toolbox', {tb.Name})),'Optimization Toolbox is required'); clear tb;
% wow, that is cool 

%% optimize
% define map  .
map = Map.getInstance();
manager = AgentManager.getInstance();
manager.reset();

m = 5; % number of agents
p = 2;

% define range for RD sensors
UWBrange = 10;
sigmarand = 10;
Dminthresh = 0*0.5*UWBrange;

% define exit condition
isrigid = 0;
feasiblefound = 0;
feasibleimproved = 0;

% leader
leaderID = 1;

% init
iter = 0;
iteropt = 0;
optattempt = 0;
maxattempt = 1000;
targetfeasible = 100;
targetimproved = 10;
Jold = Inf;

% start counter
tic
disp('Find initial condition')
while (optattempt<maxattempt) && (feasibleimproved<targetimproved) && (feasiblefound<targetfeasible) %&& iter < 1

    % iter
    iter = iter + 1;    

    % disp
    clc
    disp(['Feasible improved ', num2str(feasibleimproved)]);
    disp(['Feasible found ', num2str(feasiblefound)]);
    disp(['X0 attempt ', num2str(iter)]);

    % reset manager
    manager.reset();
    manager.WS.eigThresh = 1e-5;
    manager.WS.p = p;
    manager.WS.m = m;
    manager.WS.Dminthresh = Dminthresh;

    % define initial condition
    agents_pos = zeros(m,p);
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

    % assign positions of initial team
    for i = 1:m
        if i==leaderID
            manager.createAgent(agents_pos(i,:),1,'team_leader'); %create the leader
        else
            manager.createAgent(agents_pos(i,:),1,'team_mate'); %create the followers
        end        
    end

    % get the team you just created
    team = manager.team_list{1};

    % get all agents from the tea6m 
    agents = {team.team_mates{1:end}};
    agents = {agents{1:team.leader.agent_number-1} team.leader agents{team.leader.agent_number:end}};

    % set sensors
    for i = 1:m        
        agents{i}.sensors.UWB = Sensor(agents{i}.agent_number,'range',UWBrange);        
    end     

    % get agents summary tables
    [los_table,agents_list] = calcLosMap(agents,'UWB');

    % get distances
    if ~isempty(los_table)
        D = los_table(:,5:5+p-1) - los_table(:,5+p:5+2*p-1);
        Dmin = min(vecnorm(D,2,2));
        Dmax = max(vecnorm(D,2,2));
    else
        Dmin = 0;
    end

    % get rigidity matrix 
    R = calcRigitdyMatrix(los_table,agents_list);

    % get nonzero eigs
    e = eig(R'*R);   
    pos = find(abs(e) < manager.WS.eigThresh);   

    % exit condition
    if (numel(pos) == 3)
        isrigid = 1;   
    else
        isrigid = 0;
    end

    % DminThresh
    DminThresh = manager.WS.Dminthresh;
    DmaxThresh = 2*agents{1}.sensors.UWB.max_range;

    A = calcAdjacencyMatrix(los_table,agents_list);
    [allConn, A] = agents{1}.checkConnectivity(A);

    % now run optimization
    if (allConn) && (Dmin > DminThresh)
        
        % update
        iteropt = iteropt + 1;
        optattempt = iteropt;

        % disp
        disp(['optimization attempt ', num2str(iteropt)]);        

        % copy initial condition as second team: this will be the one we optimize on
        for i = 1:m
            if i==leaderID                
                manager.createAgent(agents_pos(i,:),2,'team_leader'); %create the leader
            else
                manager.createAgent(agents_pos(i,:),2,'team_mate'); %create the followers
            end
        end
        
        % get the team you just created
        teamInit = manager.team_list{2};
        
        % get all agents from the team 
        agentsInit = {teamInit.team_mates{1:end}};
        agentsInit = {agentsInit{1:teamInit.leader.agent_number-1-m} teamInit.leader agentsInit{teamInit.leader.agent_number-m:end}};
        
        % set sensors
        for i = 1:m        
            agentsInit{i}.sensors.UWB = Sensor(agentsInit{i}.agent_number,'range',UWBrange);        
        end        
        
        X0 = reshape(agents_pos',size(agents_pos,1)*size(agents_pos,2),1);
        J0 = cost_function(X0,p);

        % define lower bounds
        LB = repmat(map.map_span(:,1),m,1);
        
        % define upper bounds
        UB = repmat(map.map_span(:,2),m,1);
        
        % optimization - patternseacrh
        options = optimoptions( 'patternsearch', ...
                                'MeshTolerance',1e-10, ...
                                'InitialMeshSize',1e2, ...
                                'UseParallel', false, ...
                                'MaxFunctionEvaluations', 1e10, ...
                                'ConstraintTolerance', 1e-1, ...
                                'StepTolerance', 1e-10, ...
                                'Cache', 'On', ...
                                'MaxTime', 30, ...
                                'Algorithm','classic');       
        
        % optimization - fmincon
        % options = optimset( 'Display','off', ...
        %                     'Algorithm','interior-point', ...                                                        
        %                     'TolX', 1e-10, ...
        %                     'TolFun', 1e-10, ...
        %                     'MaxFunEvals', 1e10, ...
        %                     'MaxIter', 1e3);
        
        % init counter
        tic
        disp('Optimizing')
        [Xtmp, J, EXITFLAG] = patternsearch(                     ...
                            @(x)cost_function(x,p), ...
                            X0,                     ...
                            [],                     ...
                            [],                     ...
                            [],                     ...
                            [],                     ...
                            Inf*LB,                   ...
                            Inf*UB,                   ...
                            @(x)nonlcon(x,p), ...
                            options);

        % get the team you just optimized
        teamOpt = manager.team_list{1};
        
        % get all agents from the team 
        agentsOpt = {teamOpt.team_mates{1:end}};
        agentsOpt = {agentsOpt{1:teamOpt.leader.agent_number-1} teamOpt.leader agentsOpt{teamOpt.leader.agent_number:end}};

        if EXITFLAG ~= -2
            % found a feasible solution
            feasiblefound = feasiblefound + 1;

            % is it better than before?
            if J < Jold
                feasibleimproved = feasibleimproved + 1;

                % store stuff  
                J0old = J0;   
                X0old = X0;                             
                [los_table0_old,agents_list0_old] = calcLosMap(agentsInit,'UWB');    
                R0_old = calcRigitdyMatrix(los_table0_old,agents_list0_old);
                e0_old = eig(R0_old'*R0_old);                

                Jold = J;   
                Xold = Xtmp;   
                [los_table_old,agents_list_old] = calcLosMap(agentsOpt,'UWB');    
                R_old = calcRigitdyMatrix(los_table_old,agents_list_old);
                e_old = eig(R_old'*R_old);                
            end        
        end

    end
    
end
topt = toc;

%% reshape agents positions
% set initial condition
X0tmp = reshape(X0old,p,floor(numel(X0old)/p))';
tmp = X0tmp(leaderID,:);
X0tmp = X0tmp - tmp;

% set optimized condition
Xtmp = reshape(Xold,p,floor(numel(Xold)/p))';
tmp = Xtmp(leaderID,:);
Xtmp = Xtmp - tmp;

% reset everything and recreate teams
% reset manager
manager.reset();
manager.WS.eigThresh = 1e-5;
manager.WS.p = p;
manager.WS.m = m;           
manager.WS.Dminthresh = Dminthresh;

% assign positions of optimized team
for i = 1:m
    if i==leaderID
        manager.createAgent(Xtmp(i,:),1,'team_leader'); %create the leader        
    else        
        manager.createAgent(Xtmp(i,:),1,'team_mate'); %create the followers
    end        
end

% assign position for init team
for i = 1:m
    if i==leaderID        
        manager.createAgent(X0tmp(i,:),2,'team_leader'); %create the leader
    else
        manager.createAgent(X0tmp(i,:),2,'team_mate'); %create the followers        
    end        
end

% get the team you just created
teamOpt = manager.team_list{1};
teamInit = manager.team_list{2};

% get all agents from the team optimized 
agentsOpt = {teamOpt.team_mates{1:end}};
agentsOpt = {agentsOpt{1:teamOpt.leader.agent_number-1} teamOpt.leader agentsOpt{teamOpt.leader.agent_number:end}};

% get all agents from the team initialized
agentsInit = {teamInit.team_mates{1:end}};
agentsInit = {agentsInit{1:teamInit.leader.agent_number-1-m} teamInit.leader agentsInit{teamInit.leader.agent_number-m:end}};


% set sensors
for i = 1:m        
    agentsOpt{i}.sensors.UWB = Sensor(agentsOpt{i}.agent_number,'range',UWBrange);        
    agentsInit{i}.sensors.UWB = Sensor(agentsInit{i}.agent_number,'range',UWBrange);        
end     

              
%% plot section

warning('off','all')

% LOS map
teams = manager.getAllTeams();

scaleaxis = 1.2;

% team 2 - init condition
f1 = figure(1);
hold on; box on; grid on; 
set(gca,'fontsize', 20);

% set subplots
s1 = subplot(1,2,1);
hold on; box on; grid on; 
set(gca,'fontsize', 20);

% set map bounds
fill([  map.map_span(1,1) map.map_span(1,1) map.map_span(1,2) map.map_span(1,2)], ...
     [  map.map_span(2,1) map.map_span(2,2) map.map_span(2,2) map.map_span(2,1)],...
     [  0.5 0.2 0.6], ...
     'FaceAlpha',0.1, ...
     'LineWidth',1.5);

s2 = subplot(1,2,2);
hold on; box on; grid on; 
set(gca,'fontsize', 20);

% set map bounds
fill([  map.map_span(1,1) map.map_span(1,1) map.map_span(1,2) map.map_span(1,2)], ...
     [  map.map_span(2,1) map.map_span(2,2) map.map_span(2,2) map.map_span(2,1)],...
     [  0.5 0.2 0.6], ...
     'FaceAlpha',0.1, ...
     'LineWidth',1.5);

teamInit.plotTeam(s1,9);
drawLosMap(los_table0_old,s1,9);
xlim(scaleaxis*map.map_span(1,:)); ylim(scaleaxis*map.map_span(2,:));
axis equal

% team 2 - optimized
teamOpt.plotTeam(s2,2);
drawLosMap(los_table_old,s2,2);
xlim(scaleaxis*map.map_span(1,:)); ylim(scaleaxis*map.map_span(2,:));
axis equal

xlabel('X axis')
ylabel('Y axis')

warning('on','all')

%% silly
load('handel.mat')
sound(y,Fs);