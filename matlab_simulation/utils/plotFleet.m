%% Plot fleet
% file: plotFleet.m
% author: Federico Oliva 
% date: 20/02/2024
% description: plot fleet with localization and links
function f1 = plotFleet(f1,gifFile,flag,delay,repeat,agents_pos_original)

    if flag 
        % get manager
        manager = AgentManager.getInstance;
    
        % get map
        map = Map.getInstance;
    
        % get agents
        team = manager.getAllTeams;
               
        % get team localized
        agents = {team{1}.team_mates{1:end}};
        agents = {agents{1:team{1}.leader.agent_number-1} team{1}.leader agents{team{1}.leader.agent_number:end}};
    
        % get agents positions        
        for i=1:numel(agents)            
            agents_pos(agents{i}.agent_number,:) = agents{i}.location;
            agents_pos_est(agents{i}.agent_number,:) = agents{i}.location_est;
        end       
    
        clf;
        hold on; box on; grid on; 
        set(gca,'fontsize', 20);
    
        scaleaxis = 1.0;
        xlim(scaleaxis*map.map_span(1,:)); ylim(scaleaxis*map.map_span(2,:));
        % axis equal
        xlabel('X axis'); ylabel('Y axis');
        
        % % MAP BOUNDS
        % fill([  map.map_span(1,1) map.map_span(1,1) map.map_span(1,2) map.map_span(1,2)], ...
        %      [  map.map_span(2,1) map.map_span(2,2) map.map_span(2,2) map.map_span(2,1)],...
        %      [  0.5 0.2 0.6], ...
        %      'FaceAlpha',0.1, ...
        %      'LineWidth',1.5);

        % leader FOV
        center = agents{team{1}.leader.agent_number}.location;
        circle(center(1),center(2),3);        
        axis square

        % plot originals
        if ~isempty(agents_pos_original)
            h5 = plot(agents_pos_original(:,1),agents_pos_original(:,2), 'o', 'MarkerSize',7,'LineWidth',2,'Color',[0.8 0 0],'MarkerFaceColor',[0.8 0 0]);
        end
        
        % SET ID
        ID = (1:size(agents_pos,1))';
        
        % plot all teammates + graphic info
        h1 = text(1*agents_pos(:,1)+0.3,1*agents_pos(:,2), ...                    
                    cellstr(num2str(ID)), ...                    
                        'FontSize',20, ...
                        'Color',[0.5 0 0]);    
    
        % plot estimations
        h2 = plot(agents_pos_est(:,1),agents_pos_est(:,2), 'bo', 'MarkerSize',14,'LineWidth',3);
        
        % draw los map for UWB        
        [los_UWB,~] = calcLosMap(agents,'UWB');                
        h3 = drawLosMap(los_UWB,f1,9);
        
    
        % draw los map for CAM
        [los_CAM,~] = calcLosMap(agents,'CAM');
        h4 = drawLosMap(los_CAM,f1,10);        

        % store gif
        if ~isempty(gifFile)
            for i=1:repeat
                GifStore(f1,gifFile,delay);
            end
        end

    end

end

function h = circle(x,y,r)
    hold on
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    h = plot(xunit, yunit, 'Color',[0 .5 0 0.3], 'LineWidth',2);    
end