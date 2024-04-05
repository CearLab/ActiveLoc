%% Compute Coverage
% file: computeCoverage.m
% author: Federico Oliva 
% date: 21/04/2024
% description: compute coverage of a team (Monte Carlo approach)
function C = computeCoverage(Xagents,Niter,Dthresh,plotflag)

    %%% define map  .
    map = Map.getInstance();
    
    %%% create agent manager
    manager = AgentManager.getInstance();    
    Nagent = size(Xagents,1);

    % store
    Pstore = zeros(Niter,manager.WS.p+1);

    % Monte Carlo approach
    for ii=1:Niter

        % random         
        P = zeros(1,manager.WS.p);
        for jj=1:manager.WS.p
            a = 1*map.map_span(jj,1);
            b = 1*map.map_span(jj,2);
            P(1,jj) = a + (b-a).*rand(1);
        end

        % get all distances
        D = vecnorm(Xagents - P,2,2);
        Dmin = min(D);
        
        if Dmin <= Dthresh
            Pstore(ii,:) = [P 1];
        else
            Pstore(ii,:) = [P 0];
        end

    end

    % count ones
    pos1 = find(Pstore(:,3) == 1);
    Pones = Pstore(pos1,1:2);

    % count zeros
    pos0 = find(Pstore(:,3) == 0);
    Pzeros = Pstore(pos0,1:2);

    C = numel(pos1)/Niter;

    % plot
    if plotflag

        f1 = figure(1);
        hold on; box on; grid on;
        H = gca;
        H.LineWidth = 2;

        % MAP BOUNDS
        fill([  map.map_span(1,1) map.map_span(1,1) map.map_span(1,2) map.map_span(1,2)], ...
             [  map.map_span(2,1) map.map_span(2,2) map.map_span(2,2) map.map_span(2,1)],...
             [  0.5 0.2 0.6], ...
             'FaceAlpha',0.1, ...
             'LineWidth',1.5);

        % SET ID
        ID = (1:Nagent)';

        % plot all teammates + graphic info        
        h1 = text(1*Xagents(:,1)+0.3,1*Xagents(:,2), ...                    
                    cellstr(num2str(ID)), ...                    
                        'FontSize',20, ...
                        'Color',[0.5 0 0]); 

        % plot estimations
        h2 = plot(Xagents(:,1),Xagents(:,2), 'bo', 'MarkerSize',4,'LineWidth',2,'MarkerFaceColor','b');

        % plot agents coverage
        for ii=1:Nagent

                % map area
                pos = Dthresh*[-1 -1; 2 2]; 
                pos = [Xagents(ii,:) + pos(1,:) pos(2,:)];
                rectangle('Position',pos,'Curvature',[1 1],'FaceColor',[0.65 0.84 0.65 0.2],'LineWidth',2);    
        end

        % fill Pstore
        plot(Pones(:,1),Pones(:,2),'k.','LineWidth',2);
        plot(Pzeros(:,1),Pzeros(:,2),'r.','LineWidth',2);

    end

end