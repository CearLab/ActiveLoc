%% test section
clc;clear all;close all;

% define map  .
map = Map.getInstance();
manager = AgentManager.getInstance();

n = 1e5; % number of runs
m = 4; % number of agents
p = 2; % plane

% init 
eig_list = [];
std_list = [];
X_store = [];

% counter
i = 0;

% random pick of the agent position
% agents_fixed = rand(m,2)*16 - 8;     


% cycle until you get the number of rigid montecarlo samples you need
while i < n

    % start clock
    tic

    % reset manager
    manager.reset();

    % random pick of the agent position
    agents_pos = rand(m,2)*16 - 8;   

    % simple translations of the agents
    offs = randn(1,2)*8;
    % agents_pos = agents_fixed + offs;

    % create agents
    for ii = 1:m
        manager.createAgent(agents_pos(ii,:),1,'team_mate'); %create the agents
    end 

    % get all agents
    agents = manager.getAllAgent();

    % LOS calculations
    [los_table,agents_list] = calcLosMap(agents);

    % get rigidity matrix
    R = calcRigitdyMatrix(los_table,agents_list);

    % first check rigidity
    etmp = eig(R'*R);

    % get # nnz elements
    pos = find(abs(etmp) < 1e-10);

    % flag for rigidity 
    isrigid = (numel(pos)==3);

    % if framework is rigid the accept it 
    if isrigid

        % increase counter
        i = i + 1;

        % update store
        eig_list(i,:) = etmp;
        std_list(i,:) =  std(agents_pos,1);
        X_store(:,i) = reshape(agents_pos',size(agents_pos,1)*size(agents_pos,2),1);
        % offs_store(i,:) = offs;

        % info display
        % disp([num2str(i) ': ' num2str(toc)]);

    end    
end

%% post process

% get rigidity eigenvalue
lambda4= eig_list(:,4); 

[lambda4, pos_sort] = sort(lambda4); 
std_list = std_list(pos_sort,:);
X_store = X_store(:,pos_sort);
% offs_store = offs_store(pos_sort,:);

% get entropy of the formations
std_norm = sqrt(std_list(:,1).^2 + std_list(:,2).^2);

% get average std
std_mean = mean(std_norm);

% select the fiducial line
fid = (std_norm - std_mean);

% set the fiducial interval
delta = 0.01;

% get all formations within fiducial
pos = find(abs(fid) < delta );

% get interesting formations
lambda_fid = lambda4(pos);
std_fid = std_list(pos,:);
X_fid = X_store(:,pos);
% offs_fid = offs_store(pos,:);

%% plot(

delete('SimAnimatedF2.gif')
delete('SimAnimatedF1.gif')

f1 = figure(1);
hold on; box on; grid on;
set(gca,'fontsize', 20);

plot(std_norm,lambda4,'b.');
ylabel('\lambda_4');xlabel('std(p)');

f2 = figure(2);
subplot(2,1,1)
hold on; box on; grid on; 
set(gca,'fontsize', 20);
xlabel('Iteration'); ylabel('\lambda_4');
% set animatedline
l1 = animatedline('Color','b','LineWidth',1.5);
subplot(2,1,2)
hold on; box on; grid on; 
set(gca,'fontsize', 20);
xlabel('Iteration'); ylabel('metric');
% set animatedline
l2 = animatedline('Color',[0.5 0 0],'LineWidth',1.5);

% scaleaxis = 1.2;
% xlim(scaleaxis*map.map_span(1,:)); ylim(scaleaxis*map.map_span(2,:));
% axis equal
% xlabel('X axis'); ylabel('Y axis');

% set map bounds
% fill([  map.map_span(1,1) map.map_span(1,1) map.map_span(1,2) map.map_span(1,2)], ...
%      [  map.map_span(2,1) map.map_span(2,2) map.map_span(2,2) map.map_span(2,1)],...
%      [  0.5 0.2 0.6], ...
%      'FaceAlpha',0.1, ...
%      'LineWidth',1.5);
% 
% Pos =[  map.map_span(1,1) , ...
%         map.map_span(2,1) , ...
%         map.map_span(2,2)-map.map_span(2,1), ...
%         map.map_span(1,2)-map.map_span(1,1)];
% 
% rectangle(  'Position',Pos, ...
%             'Curvature',[1 1], ...
%             'FaceColor',[0.5 0.2 0.6 0.5], ...
%             'EdgeColor','b',...
%             'LineWidth',1.5);

% set positions
pos1 = get(f1,'Position'); % get position of Figure(1) 
set(f1,'Position', pos1 - [pos1(3)/2,0,0,0]) % Shift position of Figure(1)
pos2 = get(f2,'Position'); % get position of Figure(2) 
set(f2,'Position', pos2 + [pos2(3)/2,0,0,0]) % Shift position of Figure(2)


for i=1:numel(lambda_fid)

    % plot norm
    figure(f1)
    plot(norm(std_fid(i,:)),lambda_fid(i),'r+'); 

    % figure f2
    figure(f2)
   
    % reshape agents positions
    x = X_fid(:,i);
    loc = reshape(x,p,floor(numel(x)/p))';
    N = size(loc,1);

    % plot the mean in the plane
    avg = mean(loc);
    P0 = N*[avg(1)^2, avg(2)^2];
    P1 = [sum(loc(:,1).^2), sum(loc(:,2).^2)];
    P2 = [sum(loc(:,1)*avg(1)), sum(loc(:,2)*avg(2))];

    % complex one
    for a = 1:N
        tmpA = loc(a,:);
        for b=1:N
            tmpB(b,:) = tmpA - loc(b,:);
        end
        tmpT = tmpA.*tmpB;
        T(a,:) = sum(tmpT,1);
    end
    std_square = sum(T,1)/(N^2);

    % test
    % metr = 0*P0/N + 0*P1/N -1*2/N*P2;
    % metr = norm(offs_fid(i,:));
    metr = min(std_fid(i,:))/max(std_fid(i,:));
    % metr = avg(1);
    
    % plot cost function              
    addpoints(l1,i,lambda_fid(i));
    ylim('auto')
    % plot condition number       
    addpoints(l2,i,metr);
    ylim('auto')
    
    % get ID
    % ID = (1:size(loc,1))';
    
    % plot all teammates + graphic info
    % h1 = text(1*loc(:,1),1*loc(:,2), ...                    
    %             cellstr(num2str(ID)), ...                    
    %                 'FontSize',20, ...
    %                 'Color',[0.5 0 0]);
    
    % reassign agents position
    % for j=1:numel(agents)
    %     agents{j}.location = loc(j,:);
    % end

    % get agents summary tables
    % [los_table,~] = calcLosMap(agents);
    
    % draw los map
    % h2 = drawLosMap(los_table,f2,9);

    drawnow
    pause(1e-3);

    % gif
    exportgraphics(f1,'SimAnimatedF1.gif','Append',true);

    % gif       
    exportgraphics(f2,'SimAnimatedF2.gif','Append',true);

    % clear figures
    if i < numel(lambda_fid)
        % delete(h1)
        % for j=1:numel(h2)
        %     delete(h2{j})
        % end
    end

end

