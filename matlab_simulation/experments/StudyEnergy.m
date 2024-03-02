%% test section
clc;clear;close all;
warning off
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

% we first define our initial team
while ~isOK

    % reset manager
    manager.reset();

    % random pick of the agent position
    % agents_pos = rand(m,2)*16 - 8;  

    % for the sake of simplicity we now consider only one ad-hoc formation
    % d is the distance between nodes
    % we start from a non-rigid formation
    d = 4;
    agents_pos = [  -d,  -d;     ...
                    -d,  +d;     ...
                    +d,  -d;     ...
                    +d,  +d];

    % define leader
    leaderID = 1;

    % create agents
    for ii = 1:m        
        if ii==leaderID 
            manager.createAgent(agents_pos(ii,:),1,'team_leader');
        else
            manager.createAgent(agents_pos(ii,:),1,'team_mate');
        end
    end 

    % get all agents
    agents = manager.getAllAgent();

    % set sensors
    % the leader also has a camera, but for now we are not using it. The
    % number at the end is the sensor maximum range
    for i = 1:m
        if i == leaderID
            agents{i}.sensors.CAM = Sensor(agents{i}.agent_number,'range',5);
            agents{i}.sensors.UWB = Sensor(agents{i}.agent_number,'range',10);
        else
            agents{i}.sensors.UWB = Sensor(agents{i}.agent_number,'range',10);
        end
    end

    % we first check whether the constraints are met by this formation:
    % 1) the fleet is not infinitesinally rigid
    % 2) there is a minimum distance between nodes
    % check nonlconEnergy for more info
    [c, ceq] = nonlconEnergy(agents_pos,p);

    % are the constraints met?
    % I am interested only in c < 0, namely the min distance. Rigidity is
    % not a requiremement for this first formation
    isOK = prod(c < -1e-10);    % checj all inequalities are less than zero

    if isOK        
    
        % get all agents
        agents = manager.getAllAgent();
    
        % LOS calculations (get the LOS table of each agent). In this case
        % it will be a 4 rows table (it's a parallelogram)
        [los_table,agents_list] = calcLosMap(agents,'UWB');            

        % store the positions as a single columns
        X_store = reshape(agents_pos',size(agents_pos,1)*size(agents_pos,2),1);

    end

end

%% generate measurements

% true distance between agents in LOS
Dtrue = calcDistanceMatrix(los_table,agents_list);

% now I find the IDs of bars and struts/cables
barsID = find((los_table(:,1) == manager.team_list{1}.leader.agent_number) | (los_table(:,2) == manager.team_list{1}.leader.agent_number));
RDID = find((los_table(:,1) ~= manager.team_list{1}.leader.agent_number) & (los_table(:,2) ~= manager.team_list{1}.leader.agent_number));

% SLAM meas matrix
% here I am saying that SLAM distances are those with the camera and so are
% very precise. I don't put noise on SLAM, but rather on UWB.
Dslam = Dtrue;

% I remove the data of the RD
for i = 1:numel(RDID)
    Dslam(los_table(RDID(i),1),los_table(RDID(i),2)) = 0;
    Dslam(los_table(RDID(i),2),los_table(RDID(i),1)) = 0;
end

% basically, who is in LOS of the leader has a BAR, all the others a
% cable/strut.

% noise on UWB
RDsigma = 1*0.2;
RDnoise = RDsigma*randn(manager.agents_counter);

% boolean mask (what are the distances)
mask = Dtrue > 0;

% RD meas matrix
DRD = Dtrue + (RDnoise.*mask);

% now I remove the values which are in the leader LOS (already in Dslam)
for i = 1:numel(barsID)
    DRD(los_table(barsID(i),1),los_table(barsID(i),2)) = 0;
    DRD(los_table(barsID(i),2),los_table(barsID(i),1)) = 0;
end

% in total we have Dmeas
Dmeas = Dslam + DRD;

% now we go for the cables and struts 

% find rows and cols where there are RD measures (like in mask)
% could have got them from los but i'm lazy
[rtmp,ctmp] = find(DRD~=0);

% init Dcable and Dstrut which are the lower and upper bounds for those
% measures with uncertainty
Dcable = zeros(size(DRD));
Dstrut = zeros(size(DRD));

% cycle over the nonero distances measured
for i=1:numel(rtmp)

    % the cable is an upper bound: max of the two symmetric measurements
    % but less than the sensing range
    Dcable(rtmp(i),ctmp(i)) = min(max(DRD(rtmp(i),ctmp(i)), DRD(ctmp(i),rtmp(i))) + 2*RDsigma, 0.99*agents{leaderID}.sensors.UWB.max_range);

    % the strut is a lower bound: min of the two symmetric measurements
    % but greater than zero (of course)
    Dstrut(rtmp(i),ctmp(i)) = max(min(DRD(rtmp(i),ctmp(i)), DRD(ctmp(i),rtmp(i))) - 2*RDsigma, 0);    
end

% force symmetry
Dcable = (Dcable+Dcable')/2;
Dstrut = (Dstrut+Dstrut')/2;
Dbars = Dslam;

%% optimization

% compute initial energy 

% find the mask with actual measures
WM0 = double(Dmeas > 0);
w0 = 1*nonzeros(WM0);
[r, c] = find(WM0 ~= 0);
for i=1:numel(r)
    WM0(r(i),c(i)) = w0(i);
end

% put initial positions in column (true ones)
PM0 = agents_pos;
p0 = reshape(PM0',size(PM0,1)*size(PM0,2),1);

% define initial condition
X0 = [p0; w0];

% store
manager.WS.w0 = w0;
manager.WS.p0 = p0;
manager.WS.p = p;

% get subLOS for bars and RD
bars = los_table(barsID,:);
RD = los_table(RDID,:);

% now compute the energy
E0 = 0; % init
% according to connelly, no consitions on wij if we consider bars. Let's
% consider bars to begin with
for i=1:numel(w0)       
    % distance here
    D = norm(PM0(r(i),:) - PM0(c(i),:))^2;
    % energy
    E0 = E0 + WM0(r(i),c(i))*D.^2;
end

% ok now we optimize
% let's try yalmip
% clear yalmip
yalmip('clear');  

% define vars

% define vector (P,W)
X = sdpvar(numel(X0),1);
assign(X,X0);

% extract data
P = X(1:p*N);
W = X(p*N+1:end);

% reshape positions in matrix
PM = reshape(P,p,floor(numel(P)/p))';

% let's start with the constraints
Constraints = [];

% leader position constraint: i want the leader to stay there, to avoid
% translations
Constraints = [ Constraints, ... 
                PM(leaderID,:) == X0(leaderID,:)]; 


% bars constraints: the distances of the bars must be always the same as in
% Dslam
for i = 1:size(bars,1)
    deltaPbars(i,:) = PM(bars(i,1),:) - PM(bars(i,2),:);
end
dbars = Dbars(bars(1,1),bars(:,2))';
Constraints = [ Constraints, ... 
                deltaPbars*deltaPbars' == dbars*dbars'  ]; 

% RD constraints, same sith bounds
for i = 1:size(RD,1)
    deltaPRD(i,:) = PM(RD(i,1),:) - PM(RD(i,2),:);
end
dcable = Dcable(RD(1,1),RD(:,2))';
dstrut = Dstrut(RD(1,1),RD(:,2))';
Constraints = [ Constraints, ... 
                dstrut*dstrut' <= deltaPRD*deltaPRD', ...
                deltaPRD*deltaPRD' <= dcable*dcable'   ]; 

% W constraints
% first set the weight matrix
for i=1:numel(r)
    WM(r(i),c(i)) = W(i);
end

% now set the constraints. I only set ~= zero, or should I set less bigger
% than?
for i=1:size(RD,1)
    Constraints = [Constraints, WM(RD(i,1),RD(i,2)) ~= 0];
end

% Define an objective, i.e. the energy
% now compute energy
E = 0;
% cycle only on the upper triangle of Distance Matrix
for i=1:numel(r)        
    D = norm(PM(r(i),:) - PM(c(i),:))^2;
    E = E + WM(r(i),c(i))*D.^2;
end

Objective = E;

% Set some options for YALMIP and solver
% very much random tests

% options = [];
% options = sdpsettings('verbose',1,'solver','quadprog','quadprog.maxiter',100);
options = sdpsettings('solver','bmibnb');
% options = sdpsettings('solver','mosek');
% options = sdpsettings('solver','gurobi');


% Solve the problem
% sol = optimize(Constraints,Objective,options);
sol.problem = 0;

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

% now assign the solution to the team number 2. All as in the beginning
Psol = solution(1:p*N);
Psol = reshape(Psol,p,floor(numel(Psol)/p))';

% create the agents
for i = 1:m
    if i==leaderID
        manager.createAgent(Psol(i,:),2,'team_leader'); 
    else
        manager.createAgent(Psol(i,:),2,'team_mate'); 
    end
end

% get teams
teams = manager.getAllTeams();

% get the agents of the initial condition    
agents0 = {teams{2}.team_mates{1:end}};
agents0 = {agents0{1:teams{2}.leader.agent_number-1-m} teams{2}.leader agents0{teams{2}.leader.agent_number-m:end}};   

% set sensors
for i = 1:m
    if i == leaderID
        agents0{i}.sensors.CAM = Sensor(agents0{i}.agent_number,'range',5);
        agents0{i}.sensors.UWB = Sensor(agents0{i}.agent_number,'range',10);
    else
        agents{i}.sensors.UWB = Sensor(agents{i}.agent_number,'range',10);
    end
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
% get the agents of the initial condition    
agents = {teams{1}.team_mates{1:end}};
agents = {agents{1:teams{1}.leader.agent_number-1} teams{1}.leader agents{teams{1}.leader.agent_number:end}};   

h1 = text(1*loc(:,1),1*loc(:,2), ...                    
            cellstr(num2str(ID)), ...                    
                'FontSize',20, ...
                'Color',[0.5 0 0]);

% get agents summary tables
[los_table,~] = calcLosMap(agents,'UWB');
% draw los map
h2 = drawLosMap(los_table,f1,9);

% plot all teammates + graphic info
% get the agents of the solution   
agents = {teams{2}.team_mates{1:end}};
agents = {agents{1:teams{2}.leader.agent_number-1-m} teams{2}.leader agents{teams{2}.leader.agent_number-m:end}};   

h3 = text(1*Psol(:,1),1*Psol(:,2), ...                    
            cellstr(num2str(ID)), ...                    
                'FontSize',20, ...
                'Color',[0 0 0.5]);

% get agents summary tables
axis('auto');
[los_table,~] = calcLosMap(agents,'UWB');
h4 = drawLosMap(los_table,f1,2,2);

    


