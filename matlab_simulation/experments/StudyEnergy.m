%% test section
clc;clear all;close all;
rng(1);

% define map  .
map = Map.getInstance();
manager = AgentManager.getInstance();

n = 1; % number of runs
m = 4; % number of agents
p = 2; % plane
N = m;

% exit flag
isOK = 0;


% cycle until you get the number of rigid montecarlo samples you need
while ~isOK

    % reset manager
    manager.reset();

    % random pick of the agent position
    % agents_pos = rand(m,2)*16 - 8;  

    % ad hoc formation
    d = 5;
    agents_pos = [  -d,  -d;     ...
                    -d,  +d;     ...
                    +d,  -d;     ...
                    +d,  +d];

    % create agents
    for ii = 1:m        
        if ii==1 
            manager.createAgent(agents_pos(ii,:),1,'team_leader');
        else
            manager.createAgent(agents_pos(ii,:),1,'team_mate');
        end
    end 

    [c, ceq] = nonlconEnergy(agents_pos,p);
    isOK = (prod(c < -1e-10) && (ceq == 0));

    % workaround
    isOK = 1;

    if isOK        
    
        % get all agents
        agents = manager.getAllAgent();
    
        % LOS calculations
        [los_table,agents_list] = calcLosMap(agents);
    
        % get rigidity matrix
        R = calcRigitdyMatrix(los_table,agents_list);
    
        % first check rigidity
        eig = eig(R'*R);

        % store
        X_store = reshape(agents_pos',size(agents_pos,1)*size(agents_pos,2),1);

    end

end

%% generate measurements

% true distance
Dtrue = calcDistanceMatrix(los_table,agents_list);

% SLAM meas matrix
Dslam = Dtrue;

% noise
RDsigma = 1*0.2;
RDnoise = RDsigma*randn(manager.agents_counter);

% boolean mask
mask = Dtrue > 0;

% RD meas matrix
DRD = Dtrue + (RDnoise.*mask);

% cable bounds (upper)
[rtmp,ctmp] = find(DRD~=0);
Dcable = zeros(size(DRD));
Dstrut = zeros(size(DRD));
for i=1:numel(rtmp)
    Dcable(rtmp(i),ctmp(i)) = min([DRD(rtmp(i),ctmp(i)) + 2*RDsigma,DRD(ctmp(i),rtmp(i)) + 2*RDsigma,0.99*Sensor.max_range]);
    Dstrut(rtmp(i),ctmp(i)) = max([DRD(rtmp(i),ctmp(i)) - 2*RDsigma,DRD(ctmp(i),rtmp(i)) - 2*RDsigma,0]);
end
Dcable = (Dcable+Dcable')/2;
Dstrut = (Dstrut+Dstrut')/2;
Dbars = Dslam;

%% optimization

% compute initial energy 
W0 = Dtrue > 0;
barsID = (los_table(:,1) == manager.team_list{1}.leader.agent_number) | (los_table(:,2) == manager.team_list{1}.leader.agent_number);
bars = los_table(barsID,:);
RDID = (los_table(:,1) ~= manager.team_list{1}.leader.agent_number) & (los_table(:,2) ~= manager.team_list{1}.leader.agent_number);
RD = los_table(RDID,:);
W0 = double(W0);
manager.WS.W0 = W0;
manager.WS.p = p;

w0 = 0*nonzeros(W0);
p0 = reshape(agents_pos',size(agents_pos,1)*size(agents_pos,2),1);
X0 = [p0; w0];
E0 = StrutEnergy(X0);

% let's try yalmip

% clear 
yalmip('clear');  

% define vars
X = sdpvar(numel(X0),1);
assign(X,X0);
P = X(1:p*N);
w = X(p*N+1:end);
Constraints = [];

% constraints
PM = reshape(P,p,floor(numel(P)/p))';

% leader position constraint
% Constraints = [ Constraints, ... 
%                 PM(bars(1,1),:) == X0(bars(1,1),:)]; 


% bars constraints
for i = 1:size(bars,1)
    deltaPbars(i,:) = PM(bars(i,1),:) - PM(bars(i,2),:);
end
dbars = Dbars(bars(1,1),bars(:,2))';
Constraints = [ Constraints, ... 
                diag(deltaPbars*deltaPbars') == dbars.^2]; 

% RD constraints
for i = 1:size(RD,1)
    deltaPRD(i,:) = PM(RD(i,1),:) - PM(RD(i,2),:);
end
dcable = Dcable(RD(1,1),RD(:,2))';
dstrut = Dstrut(RD(1,1),RD(:,2))';
Constraints = [ Constraints, ... 
                (dstrut.^2 <= diag(deltaPRD*deltaPRD')), ...
                (diag(deltaPRD*deltaPRD') <= dcable.^2)]; 

% W constraints
% first set the weight matrix
[r, c] = find(W0 ~= 0);
for i=1:numel(r)
    W(r(i),c(i)) = w(i);
end
for i=1:size(RD,1)
    Constraints = [Constraints, W(RD(i,1),RD(i,2)) ~= 0];
end

% Define an objective
% now compute energy
E = 0;

% cycle only on the upper triangle of Distance Matrix
for i=1:numel(r)        
    D = norm(PM(r(i),:) - PM(c(i),:))^2;
    E = E + W(r(i),c(i))*D.^2;
end

Objective = E;

% Set some options for YALMIP and solver
options = [];
% options = sdpsettings('verbose',1,'solver','quadprog','quadprog.maxiter',100);
options = sdpsettings('solver','bmibnb');
% options = sdpsettings('solver','mosek');
% options = sdpsettings('solver','gurobi');


% Solve the problem
sol = optimize(Constraints,Objective,options);

% Analyze error flags
if sol.problem == 0
    % Extract and display value
    solution = value(X);
else
    disp('Hmm, something went wrong!');
    sol.info
    yalmiperror(sol.problem)
    solution = value(X);
end

% assign positions of initial team
Psol = solution(1:p*N);
Psol = reshape(Psol,p,floor(numel(Psol)/p))';

% create the agents
manager.createAgent(Psol(ii,:),2,'team_leader'); 
for ii = 2:m
    manager.createAgent(Psol(ii,:),2,'team_mate'); 
end

%% ANIMATION

% assign pos
loc = agents_pos;
% get ID
ID = (1:size(loc,1))';

% set figure
f1 = figure(1);
hold on; box on; grid on; 
set(gca,'fontsize', 20);
scaleaxis = 1.2;
xlim(scaleaxis*map.map_span(1,:)); ylim(scaleaxis*map.map_span(2,:));
axis equal
xlabel('X axis'); ylabel('Y axis');

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

% plot all teammates + graphic info
agents = {manager.team_list{1}.leader, manager.team_list{1}.team_mates{1:end}};

h1 = text(1*loc(:,1),1*loc(:,2), ...                    
            cellstr(num2str(ID)), ...                    
                'FontSize',20, ...
                'Color',[0.5 0 0]);

% get agents summary tables
[los_table,~] = calcLosMap(agents);
% draw los map
h2 = drawLosMap(los_table,f1,9);

% plot all teammates + graphic info
agents = {manager.team_list{2}.leader, manager.team_list{2}.team_mates{1:end}};

h1 = text(1*Psol(:,1),1*Psol(:,2), ...                    
            cellstr(num2str(ID)), ...                    
                'FontSize',20, ...
                'Color',[0 0 0.5]);

% get agents summary tables
axis('auto');
[los_table,~] = calcLosMap(agents);
h2 = drawLosMap(los_table,f1,9,2);

    


