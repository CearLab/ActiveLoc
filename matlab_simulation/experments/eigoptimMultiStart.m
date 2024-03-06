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

m = 7; % number of agents
p = 2;

% define range for RD sensors
UWBrange = 8;
sigmarand = 10;

% define exit condition
isrigid = 0;
feasiblefound = 0;
feasibleimproved = 0;

% leader
leaderID = 1;

% init
iter = 0;
iteropt = 0;
targetfeasible = 100;
targetimproved = 5;
Jold = Inf;

% start counter
tic
disp('Find initial condition')
while (feasibleimproved<targetimproved) && (feasiblefound<targetfeasible) %&& iter < 1

    % iter
    iter = iter + 1;

    % disp
    clc
    disp(['Feasible improved ', num2str(feasibleimproved)]);
    disp(['Feasible found ', num2str(feasiblefound)]);
    disp(['X0 attempt ', num2str(iter)]);

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

    % reset manager
    manager.reset();

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
    else
        Dmin = 0;
    end

    % get rigidity matrix 
    R = calcRigitdyMatrix(los_table,agents_list);

    % get nonzero eigs
    e = eig(R'*R);

    % get # nnz elements
    pos = find(abs(e) < 1e-10);

    % set Dminthresh
    Dminthresh = 0*0.25*max(max(map.map_span));
    if isinf(Dminthresh)
        Dminthresh = 0;
    end

    % exit condition
    if (numel(pos) < 4) && (Dmin > Dminthresh)
        isrigid = 1;   
    else
        isrigid = 0;
    end

    % now run optimization
    if isrigid
        
        % update
        iteropt = iteropt + 1;

        % disp
        disp(['optimization attempt ', num2str(iteropt)]);

        % init manager
        manager.WS.p = p;
        manager.WS.m = m;
        
        % copy initial condition as second team: this will be the one we optimize on
        for i = 1:m
            if i==leaderID
                manager.createAgent(agents_pos(i,:),2,'team_leader'); %create the leader
            else
                manager.createAgent(agents_pos(i,:),2,'team_mate'); %create the followers
            end
        end
        
        % get the team you just created
        team = manager.team_list{2};
        
        % get all agents from the team 
        agents = {team.team_mates{1:end}};
        agents = {agents{1:team.leader.agent_number-1-m} team.leader agents{team.leader.agent_number-m:end}};
        
        % set sensors
        for i = 1:m        
            agents{i}.sensors.UWB = Sensor(agents{i}.agent_number,'range',UWBrange);        
        end

        % test J0

        % set store options for cost function
        manager.WS.J = [];
        manager.WS.X = [];
        manager.WS.CN = [];
        
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
                                'ConstraintTolerance', 1e-4, ...
                                'StepTolerance', 1e-10, ...
                                'Cache', 'On', ...
                                'MaxTime', 300, ...
                                'Algorithm','classic');
        
        % optimization - fmincon
        % options = optimoptions( 'fmincon', ...
        %                         'Algorithm','interior-point', ...
        %                         'MaxFunctionEvaluations',1e5, ...
        %                         'MaxIterations',1e3, ...
        %                         'OptimalityTolerance',1e-10, ...
        %                         'StepTolerance', 1e-10, ...
        %                         'FunctionTolerance',1e-10);
        
        % optimization - fmincon
        % options = optimset( 'Display','off', ...
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
                            1*LB,                   ...
                            1*UB,                   ...
                            @(x)nonlcon(x,p), ...
                            options);

        if EXITFLAG ~= -2
            feasiblefound = feasiblefound + 1;
            if J < Jold
                feasibleimproved = feasibleimproved + 1;
                Jold = J;
                X = Xtmp;
            end        
        end

    end
    
end
topt = toc;

% rotation and translation (nodes 1-2)
% reshape agents positions
X0tmp = reshape(X0,p,floor(numel(X0)/p))';
Xtmp = reshape(X,p,floor(numel(X)/p))';
tmp = Xtmp(leaderID,:);
Xtmp = Xtmp - tmp;
V10 = X0tmp(1,:);
V1 = Xtmp(1,:);
DP = V1-V10;

% translation
% Xtmp = Xtmp - DP;

% init
Xbest = reshape(Xtmp',size(Xtmp,1)*size(Xtmp,2),1);

[c, ceq] = nonlcon(Xbest,p);
              
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

teams{2}.plotTeam(s1,9);
drawLosMap(los_table0,s1,9);
xlim(scaleaxis*map.map_span(1,:)); ylim(scaleaxis*map.map_span(2,:));
axis equal

% team 1 - optimized
% f1 = figure(2);
% hold on; box on; grid on;

% get the team you just created
team = manager.team_list{1};    

% get all agents from the team 
agents = {team.team_mates{1:end}};    
% insert leader in right position
agents = {agents{1:team.leader.agent_number-1} team.leader agents{team.leader.agent_number:end}};

% get agents summary tables
[los_table,agents_list] = calcLosMap(agents,'UWB');

teams{1}.plotTeam(s2,2);
drawLosMap(los_table,s2,2);
xlim(scaleaxis*map.map_span(1,:)); ylim(scaleaxis*map.map_span(2,:));
axis equal

xlabel('X axis')
ylabel('Y axis')

warning('on','all')