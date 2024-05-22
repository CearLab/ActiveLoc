%% test section
clc;clear all;close all;

% define map  .
map = Map.getInstance();
manager = AgentManager.getInstance();

n = 5000; % number of runs
m_max = 50; % number of agents
p_list = [];
log = {};
for m = 10:5:50
    eig_list = zeros(n,2*m);
    std_list = zeros(n,2);
    cov_det_list = zeros(n,1);

    
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
        cov_det_list(i,:) =  sqrt(det(cov(agents_pos)));

        disp([num2str(m) ':' num2str(i) ':' num2str(toc)]);
    end
    lambda4 = eig_list(:,4);
    std_norm = sqrt(std_list(:,1).^2 + std_list(:,2).^2);
    p_list(end+1,:) = [m,findUpperBound(cov_det_list,lambda4)];
    log(end+1,:) = {m,eig_list,agents_pos,cov_det_list};

end
save("res\m10to50_fixed.mat","p_list","m_max","log");
%%
lambda4 = eig_list(:,4);
std_norm = sqrt(std_list(:,1).^2 + std_list(:,2).^2);
%%

plot(std_norm,lambda4,'o');ylabel('\lambda_4');xlabel('std(p)');
% plot(std_list(:,2),lambda4,'o');ylabel('\lambda_4');xlabel('std(p)');
%%
figure;plot(p_list(:,1),p_list(:,2),'o');ylabel('m');xlabel('a');
hold on;grid on; box on;ylabel('a');xlabel('m');
p = polyfit(p_list(:,1), p_list(:,2), 1);

aFit = polyval(p, p_list(:,1));
plot(p_list(:,1), aFit, 'r-', 'LineWidth', 2); hold on