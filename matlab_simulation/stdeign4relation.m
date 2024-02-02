%% test section
clc;clear all;close all;

% define map  .
map = Map.getInstance();
manager = AgentManager.getInstance();

n = 200; % number of runs
m = 30; % number of agents

eig_list = [];
std_list = [];

for i = 1:n
    tic
    manager.reset();
    agents_pos = rand(m,2)*16 - 8; % random pick of the agent position
    for ii = 1:m
        manager.createAgent(agents_pos(ii,:),1,'team_mate'); %create the agents
    end 
    agents = manager.getAllAgent();
    % calculations
    [los_table,agents_list] = calcLosMap(agents);
    R = calcRigitdyMatrix(los_table,agents_list);
    eig_list(i,:) = eig(R'*R);
    std_list(i,:) =  std(agents_pos);
    disp([num2str(i) ':' num2str(toc)]);
end

%%
lambda4 = eig_list(:,4);
std_norm = sqrt(std_list(:,1).^2 + std_list(:,2).^2);
%%

plot(std_norm,lambda4,'o');ylabel('\lambda_4');xlabel('std(p)');
% plot(std_list(:,2),lambda4,'o');ylabel('\lambda_4');xlabel('std(p)');
%%
