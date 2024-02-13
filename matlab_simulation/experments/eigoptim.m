%% Eignevalues optimization
% file: eigoptim.m
% author: Federico Oliva 
% date: 01/02/2024
% description: test on lambda4 interpretation

%% init 
clc;
clear;
close all;
rng(2);
%% check toolbox dependencies 
tb = ver; assert(any(strcmp('Optimization Toolbox', {tb.Name})),'Optimization Toolbox is required'); clear tb;
%% optimize
% define map  .
map = Map.getInstance();
manager = AgentManager.getInstance();
manager.reset();

m = 20; % number of agents
p = 2;

% define exit condition
isrigid = 0;

% start counter
tic
disp('Find initial condition')
while ~isrigid

    % define initial condition
    agents_pos = randn(m,p)*8;  % random pick of the agent position
    agents_pos(:,1) = min(max(agents_pos(:,1),map.map_span(1,1)),map.map_span(1,2)); % saturate on the map span X
    agents_pos(:,2) = min(max(agents_pos(:,2),map.map_span(2,1)),map.map_span(2,2)); % saturate on the map span Ys

    % reset manager
    manager.reset();

    % assign positions of initial team
    for ii = 1:m
        manager.createAgent(agents_pos(ii,:),1,'team_mate'); %create the agents
    end

    agents = manager.getAllAgent;

    % get agents summary tables
    [los_table,agents_list] = calcLosMap(agents);

    % get distances
    if ~isempty(los_table)
        D = los_table(:,5:5+p-1) - los_table(:,5+p:end);
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
    end

    % isrigid = 1;
end
tfindinit = toc;

% copy initial condition as second team: this will be the one we optimize on
for ii = 1:m
    manager.createAgent(agents_pos(ii,:),2,'team_mate'); %create the agents
end

%% test J0

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

% optimization
options = optimoptions( 'patternsearch', ...
                        'MeshTolerance',1e-6, ...
                        'InitialMeshSize',1e2, ...
                        'UseParallel', true, ...
                        'MaxFunctionEvaluations', 1e4, ...
                        'ConstraintTolerance', 1e-4, ...
                        'StepTolerance', 1e-10);

% init counter
tic
disp('Optimizing')
[X, J] = patternsearch(                     ...
                    @(x)cost_function(x,p), ...
                    X0,                     ...
                    [],                     ...
                    [],                     ...
                    [],                     ...
                    [],                     ...
                    Inf*LB,                   ...
                    Inf*UB,                   ...
                    @(x)nonlcon(x,p));

topt = toc;

% rotation and translation (nodes 1-2)
% reshape agents positions
X0tmp = reshape(X0,p,floor(numel(X0)/p))';
Xtmp = reshape(X,p,floor(numel(X)/p))';
V10 = X0tmp(1,:);
V1 = Xtmp(1,:);
DP = V1-V10;

% translation
% Xtmp = Xtmp - DP;

% init
Xbest = reshape(Xtmp',size(Xtmp,1)*size(Xtmp,2),1);

[c, ceq] = nonlcon(Xbest,p);

               

%% plot section

% LOS map
teams = manager.getAllTeams();

scaleaxis = 1.2;

% team 1
f1 = figure(1);
hold on; box on; grid on; 
set(gca,'fontsize', 20);

% set map bounds
fill([  map.map_span(1,1) map.map_span(1,1) map.map_span(1,2) map.map_span(1,2)], ...
     [  map.map_span(2,1) map.map_span(2,2) map.map_span(2,2) map.map_span(2,1)],...
     [  0.5 0.2 0.6], ...
     'FaceAlpha',0.1, ...
     'LineWidth',1.5);

Pos =[  map.map_span(1,1) , ...
        map.map_span(2,1) , ...
        map.map_span(2,2)-map.map_span(2,1), ...
        map.map_span(1,2)-map.map_span(1,1)];

rectangle(  'Position',Pos, ...
            'Curvature',[1 1], ...
            'FaceColor',[0.5 0.2 0.6 0.5], ...
            'EdgeColor','b',...
            'LineWidth',1.5);

[los_table,~] = calcLosMap(teams{1}.team_mates);

teams{1}.plotTeam(f1,9);
drawLosMap(los_table,f1,9);
xlim(scaleaxis*map.map_span(1,:)); ylim(scaleaxis*map.map_span(2,:));
axis equal

% team 2
% f1 = figure(2);
% hold on; box on; grid on;

[los_table,~] = calcLosMap(teams{2}.team_mates);
teams{2}.plotTeam(f1,2);
drawLosMap(los_table,f1,2);
xlim(scaleaxis*map.map_span(1,:)); ylim(scaleaxis*map.map_span(2,:));
axis equal

xlabel('X axis')
ylabel('Y axis')