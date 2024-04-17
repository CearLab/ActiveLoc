%% test section
clc;clear all;close all;

% define map  .
map = Map.getInstance();
manager = AgentManager.getInstance();

n = 1e4; % number of runs
m = 10; % number of agents
p = 2; % plane

% init 
eig_list = [];
std_list = [];
X_store = [];

% counter
i = 0;

% vals
UWBrange = 8;
Dminthresh = 0*0.5*UWBrange;
leaderID = 1;

% random pick of the agent position
% agents_fixed = rand(m,2)*16 - 8;     


% cycle until you get the number of rigid montecarlo samples you need
disp('Finding dataset')
while i < n

    % start clock
    tic

    % reset manager
    manager.reset();
    manager.WS.eigThresh = 1e-5;
    manager.WS.p = p;
    manager.WS.m = m;
    manager.WS.Dminthresh = Dminthresh;

    % random pick of the agent position
    agents_pos = zeros(m,p);
    for j=1:p
        a = 0.9*map.map_span(j,1);
        b = 0.9*map.map_span(j,2);
        agents_pos(:,j) = a + (b-a).*rand(m,1);  % random pick of the agent position    
    end
    
    % translate everything to have the leader in the origin
    % dP = agents_pos(leaderID,:);
    % agents_pos = agents_pos - dP;
    
    % max over the map span
    agents_pos(:,1) = max(min(agents_pos(:,1),0.9*map.map_span(1,2)),0.9*map.map_span(1,1));
    agents_pos(:,2) = max(min(agents_pos(:,2),0.9*map.map_span(2,2)),0.9*map.map_span(2,1));

    % create agents
    % assign positions of initial team
    for j = 1:m
        if j==leaderID
            manager.createAgent(agents_pos(j,:),1,'team_leader'); %create the leader
        else
            manager.createAgent(agents_pos(j,:),1,'team_mate'); %create the followers
        end        
    end

    % get the team you just created
    team = manager.team_list{1};

    % get all agents from the tea6m 
    agents = {team.team_mates{1:end}};
    agents = {agents{1:team.leader.agent_number-1} team.leader agents{team.leader.agent_number:end}};

    % set sensors
    for j = 1:m        
        agents{j}.sensors.UWB = Sensor(agents{j}.agent_number,'range',UWBrange);        
    end     

    % LOS calculations
    [los_table,agents_list] = calcLosMap(agents,'UWB');

    % get rigidity matrix
    R = calcRigitdyMatrix(los_table,agents_list);
    Rover = R'*R;

    % first check rigidity
    etmp = eig(R'*R);

    % get # nnz elements
    pos = find(abs(etmp) < manager.WS.eigThresh);

    % flag for rigidity 
    isrigid = (numel(pos)==3);

    % check connectivity
    A = calcAdjacencyMatrix(los_table,agents_list);
    [allConn, A] = agents{1}.checkConnectivity(A);

    % if framework is rigid the accept it 
    if isrigid && allConn

        % increase counter
        i = i + 1;

        % update store
        eig_list(i,:) = etmp;
        std_list(i,:) =  std(agents_pos,1).^2;
        std_list(i,:) = sort(std_list(i,:));
        X_store(:,i) = reshape(agents_pos',size(agents_pos,1)*size(agents_pos,2),1);

        %%% std formula
        % reshape agents positions
        x = agents_pos;
        loc = reshape(x,p,floor(numel(x)/p))';      
    
        if ~isempty(los_table)

            % init
            JS = 0;        
    
            % init                                
            SXYtmp = 0;
            S2tmp = 0;
            S2T1 = 0;
            S2T2 = 0;        
    
            % cycle over all the agents
            for a=1:m
    
                % init possible deltas
                LOCb = [];
                POSb = [];
    
                % get position of single agent
                LOCa = agents_list(a,2:3);
    
                % now inner cycle
                % I'm not removing a~=b because the diff is zero anyways
                for b=1:m
                    
                    % now cycle over the LOS tab                
    
                    % find if the link does exist
                    flagA = los_table(:,3:4) == [a b];
                    flagB = los_table(:,3:4) == [b a];
                    posA = find(flagA(:,1).*flagA(:,2) == 1);
                    posB = find(flagB(:,1).*flagB(:,2) == 1);      
                    flag = min([posA posB]); 
    
                    % assign
                    if ~isempty(flag)
                        LOCb(b,:) = LOCa - agents_list(b,2:3);   
                        POSb(b,:) = agents_list(b,2:3);
                    end                                
                    
                end
    
                if ~isempty(LOCb)
    
                    % mean in the neighbors                       
                    NNeighs = nnz(LOCb(:,1))+1;                
                    muNeighs = sum(POSb)/NNeighs;                
    
                    % difference in neihcbors
                    DIFFb = sum(LOCb);
    
                    % compute terms
                    S2T1 = S2T1 + muNeighs.*(muNeighs - LOCa);                                            
                    S2T2 = S2T2 + LOCa.*DIFFb/NNeighs;    
    
                    % compute covariance
                    S2tmp = S2tmp + (LOCa - muNeighs).^2;  
                    SXYtmp = SXYtmp + prod(LOCa - muNeighs);                                   
    
                end
                            
            end        
    
            % variances terms        
            % S2 = (S2T1 + S2T2)/m;
            S2 = S2tmp/m;
            SX2(i,1) = S2(1);
            SY2(i,1) = S2(2);
            
            % symmetry term                   
            SXY(i,1) = SXYtmp/m;        
    
            % correlation
            rho2(i,1) = SXY(i,1)^2/(SX2(i,1)*SY2(i,1));   

            % connectivity
            NCONN(i,1) = size(los_table,1);
        else
            JS = 0;
            rho2(i,1) = [];            
            NCONN(i) = [];
        end  

        % Jtest
        % METR(i,1) = trace(Rover);        
        % METRtrue(i,1) = trace(Rover);

        METR(i,1) = NCONN(i,1);        
        METRtrue(i,1) = NCONN(i,1);

        
        
    end    
end

%% post process

% get rigidity eigenvalue
lambda4= eig_list(:,4); 

[lambda4, pos_sort] = sort(lambda4); 
std_list = std_list(pos_sort,:);
X_store = X_store(:,pos_sort);
SX2 = SX2(pos_sort,:);
SY2 = SY2(pos_sort,:);
SXY = SXY(pos_sort,:);
METR = METR(pos_sort,:);
METRtrue = METRtrue(pos_sort,:);
NCONN = NCONN(pos_sort,:);

% get entropy of the formations
spreadiness = METR;

% get average std
spreadiness_mean = mean(spreadiness);

% select the fiducial line
fid_spreadiness = (spreadiness - 1*spreadiness_mean);

% get symmetry
simmetry = rho2;

% get average std
simmetry_mean = mean(simmetry);

% select the fiducial line
fid_simmetry = (simmetry - 1*simmetry_mean);

% set the fiducial interval
delta = 1e-1;

% get all formations within fiducial
pos = find(abs(fid_spreadiness) < delta );

% compute sorted spreadines on selected data
lambda4Delta = lambda4(pos);
spreadinessDelta = spreadiness(pos);
symmetryDelta = simmetry(pos);



%% plot
set(0,'DefaultFigureWindowStyle','docked')

f1 = figure(1);
hold on; box on; grid on;
set(gca,'fontsize', 20);
plot(spreadiness,lambda4,'b.');
% plot(spreadiness(pos),lambda4(pos),'r+');
ylabel('\lambda_4');xlabel('spreadiness');

f2 = figure(2);
hold on; box on; grid on;
set(gca,'fontsize', 20);
plot(simmetry,lambda4,'b.');
plot(simmetry(pos),lambda4(pos),'r+');
ylabel('\lambda_4');xlabel('simmetry');

f3 = figure(3);
hold on; box on; grid on;
set(gca,'fontsize', 20);
plot(METR,lambda4,'b.');
plot(METR(pos),lambda4(pos),'r+');
ylabel('\lambda_4');xlabel('metric');

f4 = figure(4);
subplot(2,1,1)
hold on; box on; grid on; 
set(gca,'fontsize', 20);
xlabel('Iteration'); ylabel('\lambda_4');
plot(lambda4(pos),'LineWidth',2);
subplot(2,1,2)
hold on; box on; grid on; 
set(gca,'fontsize', 20);
xlabel('Iteration'); ylabel('metric');
plot(METR(pos),'LineWidth',2);
plot(METRtrue(pos),'o','LineWidth',2);

f5 = figure(5);
hold on; box on; grid on;
scatter3(spreadiness,simmetry,lambda4,'red');
flinextrap = fit([spreadiness,simmetry],lambda4,"cubicinterp");
Nsamples = [50 50];
bounds = [1 1; 1 1];
SPbounds = linspace(bounds(1,1)*min(spreadiness),bounds(1,2)*max(spreadiness),Nsamples(1));
SIbounds = linspace(bounds(2,1)*min(simmetry),bounds(2,2)*max(simmetry),Nsamples(2));
[SPinterp, SIinterp] = meshgrid(SPbounds,SIbounds);
LAMBDA4interp = flinextrap(SPinterp,SIinterp);
surf(SPinterp,SIinterp,LAMBDA4interp);
set(gca,'fontsize', 20);
xlabel('spreadiness');ylabel('simmetry');zlabel('\lambda_4');

f6 = figure(6);
subplot(3,1,1)
hold on; box on; grid on;
set(gca,'fontsize', 20);
plot(lambda4Delta,'bo--');
ylabel('\lambda_4');

subplot(3,1,2)
hold on; box on; grid on;
set(gca,'fontsize', 20);
plot(symmetryDelta,'bo--');
ylabel('\rho^2');

subplot(3,1,3)
hold on; box on; grid on;
set(gca,'fontsize', 20);
plot(spreadinessDelta,'bo--');
ylabel('trace');