%% animate plot
% file: animatePlot.m
% author: Federico Oliva 
% date: 06/02/2024
% description: create video of a sequence of plots
function animatePlot(varargin)

    % set p if not in varargin
    if isempty(varargin)
        p = 2;
    end

    % get manager
    manager = AgentManager.getInstance();

    % get teams
    teams = manager.getAllTeams();

    % get the agents
    agents = teams{2}.team_mates;

    % get map
    map = Map.getInstance();

    % get number of samples
    nsamples = numel(manager.WS.J);    

    % set figure on the plane
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

    % plot initial condition
    % get state
    x = manager.WS.X(:,1);        

    % reshape agents positions
    loc = reshape(x,p,floor(numel(x)/p))';

    % get ID
    ID = (1:size(loc,1))';

    % plot all teammates + graphic info
    text(1*loc(:,1),1*loc(:,2), ...                    
                cellstr(num2str(ID)), ...                    
                    'FontSize',20, ...
                    'Color',[0.5 0 0]);

    % reassign agents position
    for j=1:numel(agents)
        agents{j}.location = loc(j,:);
    end

    % get agents summary tables
    [los_table,~] = calcLosMap(agents);

    % draw los map
    drawLosMap(los_table,f1,9);

    % set figure for cost function
    f2 = figure(2);
    hold on; box on; grid on; 
    set(gca,'fontsize', 20);
    xlabel('Iteration'); ylabel('\lambda_4');

    % set animatedline
    l = animatedline('Color','b','LineWidth',1.5);

    % set positions
    pos1 = get(f1,'Position'); % get position of Figure(1) 
    set(f1,'Position', pos1 - [pos1(3)/2,0,0,0]) % Shift position of Figure(1)
    pos2 = get(f2,'Position'); % get position of Figure(2) 
    set(f2,'Position', pos2 + [pos2(3)/2,0,0,0]) % Shift position of Figure(2)

    % cycle and plot
    for i=1:20:nsamples

        % set figure
        figure(f1);       

        % get state
        x = manager.WS.X(:,i);        

        % reshape agents positions
        loc = reshape(x,p,floor(numel(x)/p))';                    

        % get ID
        ID = (1:size(loc,1))';

        h1 = text(1*loc(:,1),1*loc(:,2), ...                    
                cellstr(num2str(ID)), ...                    
                    'FontSize',20, ...
                    'Color','b');


        % reassign agents position
        for j=1:numel(agents)
            agents{j}.location = loc(j,:);
        end
    
        % get agents summary tables
        [los_table,~] = calcLosMap(agents);
    
        % draw los map
        h2 = drawLosMap(los_table,f1,2);

        % plot cost function
        figure(f2)
        addpoints(l,i,1/manager.WS.J(i));
        
        drawnow        
        pause(1e-1)

        delete(h1)
        for j=1:numel(h2)
            delete(h2{j})
        end

    end

    % last one
    % plot all teammates + graphic info
    figure(f1)
    text(1*loc(:,1),1*loc(:,2), ...                    
                cellstr(num2str(ID)), ...                    
                    'FontSize',20, ...
                    'Color','b');
    % draw los map
    drawLosMap(los_table,f1,2);



end