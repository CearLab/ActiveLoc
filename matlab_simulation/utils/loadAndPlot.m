%% Load And Plot
% file: loadAndPlot.m
% author: Federico Oliva 
% date: 012/03/2024
% description: load several workspaces and store info

%% clean
clear; close all; clc;

% get .mat files
filesA = dir('res/LambdaStudy/TestTraceFAST/*.mat');
filesB = dir('res/LambdaStudy/TestLambda4FAST/*.mat');
filesC = dir('res/LambdaStudy/TestSymmIncFAST/*.mat');
filesD = dir('res/LambdaStudy/TestSymmIncLambda4RHOMINFAST/*.mat');

NfilesA = numel(filesA);
NfilesB = numel(filesB);
NfilesC = numel(filesC);
NfilesD = numel(filesD);

% init
STOREA = zeros(NfilesA,7);
STOREB = zeros(NfilesB,7);
STOREC = zeros(NfilesC,7);
STORED = zeros(NfilesD,7);

%% cycle folder A
if 1
for f=1:NfilesA

    % load
    tmp = [filesA(f).folder '/' filesA(f).name];
    load(tmp);

    % store
    STOREA(f,:) = [manager.WS.Dminthresh manager.WS.Dmaxthresh manager.WS.m manager.WS.JSB e_old(4) sum(nonzeros(e_old)) size(los_table_old,1)];    

end

% split store - on Dmin and m =4
STOREA_DminN04(1).val = STOREA(find( (STOREA(:,1) == 1) & (STOREA(:,3) == 4) ), :);
STOREA_DminN04(2).val = STOREA(find( (STOREA(:,1) == 2) & (STOREA(:,3) == 4) ), :);
STOREA_DminN04(3).val = STOREA(find( (STOREA(:,1) == 3) & (STOREA(:,3) == 4) ), :);

% split store - on Dmin and m =5
STOREA_DminN05(1).val = STOREA(find( (STOREA(:,1) == 1) & (STOREA(:,3) == 5) ), :);
STOREA_DminN05(2).val = STOREA(find( (STOREA(:,1) == 2) & (STOREA(:,3) == 5) ), :);
STOREA_DminN05(3).val = STOREA(find( (STOREA(:,1) == 3) & (STOREA(:,3) == 5) ), :);


end

%% cycle folder B
if 1
for f=1:NfilesB

    % load
    tmp = [filesB(f).folder '/' filesB(f).name];
    load(tmp);

    % store
    STOREB(f,:) = [manager.WS.Dminthresh manager.WS.Dmaxthresh manager.WS.m manager.WS.JSB e_old(4) sum(nonzeros(e_old)) size(los_table_old,1)];    

end

% split store - on Dmin and m =4
STOREB_DminN04(1).val = STOREB(find( (STOREB(:,1) == 1) & (STOREB(:,3) == 4) ), :);
STOREB_DminN04(2).val = STOREB(find( (STOREB(:,1) == 2) & (STOREB(:,3) == 4) ), :);
STOREB_DminN04(3).val = STOREB(find( (STOREB(:,1) == 3) & (STOREB(:,3) == 4) ), :);

% split store - on Dmin and m =5
STOREB_DminN05(1).val = STOREB(find( (STOREB(:,1) == 1) & (STOREB(:,3) == 5) ), :);
STOREB_DminN05(2).val = STOREB(find( (STOREB(:,1) == 2) & (STOREB(:,3) == 5) ), :);
STOREB_DminN05(3).val = STOREB(find( (STOREB(:,1) == 3) & (STOREB(:,3) == 5) ), :);
end

%% cycle folder C
if 1
for f=1:NfilesC

    % load
    tmp = [filesC(f).folder '/' filesC(f).name];
    load(tmp);

    % store
    STOREC(f,:) = [manager.WS.Dminthresh manager.WS.Dmaxthresh manager.WS.m manager.WS.JSB e_old(4) sum(nonzeros(e_old)) size(los_table_old,1)];    

end

% split store - on Dmin and m =4
STOREC_DminN04(1).val = STOREC(find( (STOREC(:,1) == 1) & (STOREC(:,3) == 4) ), :);
STOREC_DminN04(2).val = STOREC(find( (STOREC(:,1) == 2) & (STOREC(:,3) == 4) ), :);
STOREC_DminN04(3).val = STOREC(find( (STOREC(:,1) == 3) & (STOREC(:,3) == 4) ), :);

% split store - on Dmin and m =5
STOREC_DminN05(1).val = STOREC(find( (STOREC(:,1) == 1) & (STOREC(:,3) == 5) ), :);
STOREC_DminN05(2).val = STOREC(find( (STOREC(:,1) == 2) & (STOREC(:,3) == 5) ), :);
STOREC_DminN05(3).val = STOREC(find( (STOREC(:,1) == 3) & (STOREC(:,3) == 5) ), :);
end

%% cycle folder D
if 1
for f=1:NfilesD

    % load
    tmp = [filesD(f).folder '/' filesD(f).name];
    load(tmp);

    % store
    STORED(f,:) = [manager.WS.Dminthresh manager.WS.Dmaxthresh manager.WS.m manager.WS.JSB e_old(4) sum(nonzeros(e_old)) size(los_table_old,1)];    

end

% split store - on Dmin and m =4
STORED_DminN04(1).val = STORED(find( (STORED(:,1) == 1) & (STORED(:,3) == 4) ), :);
STORED_DminN04(2).val = STORED(find( (STORED(:,1) == 2) & (STORED(:,3) == 4) ), :);
STORED_DminN04(3).val = STORED(find( (STORED(:,1) == 3) & (STORED(:,3) == 4) ), :);

% split store - on Dmin and m =5
STORED_DminN05(1).val = STORED(find( (STORED(:,1) == 1) & (STORED(:,3) == 5) ), :);
STORED_DminN05(2).val = STORED(find( (STORED(:,1) == 2) & (STORED(:,3) == 5) ), :);
STORED_DminN05(3).val = STORED(find( (STORED(:,1) == 3) & (STORED(:,3) == 5) ), :);
end

%% plot

% dock
set(0,'DefaultFigureWindowStyle','docked');
set(gca,'Position',get(gca,'OuterPosition'));

% width
w = 2;

% size
ms = [200 400 600];

% colors
colA = [1 0 0; 0 1 0; 0 0 1];
colB = [1 0 0; 0 1 0; 0 0 1];
colC = [1 0 0; 0 1 0; 0 0 1];

% shapes
sh = {'c' 's' 'd'};

% transparency
alpha = 0.3;

% linestyles
styles = {'bo', 'rs','kd'};

%% set figure for lambda4/trace/Nconn
if 1
figure(1);

% set figure for Trace
a1 = subplot(3,2,1);
a1.LineWidth = 2;
hold on; grid on; box on;
xlim([1.5 8.5]); ylim([0 800]);
set(gca,'fontsize', 20);
ylabel('Trace');

a2 = subplot(3,2,2);
a2.LineWidth = 2;
hold on; grid on; box on;
xlim([1.5 8.5]); ylim([0 800]);
set(gca,'fontsize', 20);

% set figure for Lambda4
a3 = subplot(3,2,3);
a3.LineWidth = 3;
hold on; grid on; box on;
xlim([1.5 8.5]); ylim([0 80]);
set(gca,'fontsize', 20);
ylabel('\lambda_4');

a4 = subplot(3,2,4);
a4.LineWidth = 2;
hold on; grid on; box on;
xlim([1.5 8.5]); ylim([0 80]);
set(gca,'fontsize', 20);

% set figure for Nconn
a5 = subplot(3,2,5);
a5.LineWidth = 2;
hold on; grid on; box on;
% set(gca, 'YScale', 'log')
xlim([1.5 8.5]); ylim([6 11]);
set(gca,'fontsize', 20);
xlabel('Dmax'); ylabel('Nconn');

a6 = subplot(3,2,6);
a6.LineWidth = 3;
hold on; grid on; box on;
xlim([1.5 8.5]); ylim([6 11]);
set(gca,'fontsize', 20);
xlabel('Dmax'); 

for f = 1:3            

    % Trace optset: N=4/5 trace
    % scatter(a1,STOREA_DminN04(f).val(:,2),STOREA_DminN04(f).val(:,6),ms(f),sh{1},'MarkerEdgeColor','black','MarkerFaceColor',colA(f,:)','MarkerFaceAlpha',alpha);    
    scatter(a1,STOREA_DminN05(f).val(:,2),STOREA_DminN05(f).val(:,6),ms(f),sh{2},'MarkerEdgeColor','black','MarkerFaceColor',colA(f,:)','MarkerFaceAlpha',alpha);    


    % Lambda4 optset: N=4/5 trace
    % scatter(a2,STOREB_DminN04(f).val(:,2),STOREB_DminN04(f).val(:,6),ms(f),sh{1},'MarkerEdgeColor','black','MarkerFaceColor',colA(f,:)','MarkerFaceAlpha',alpha);    
    scatter(a2,STOREB_DminN05(f).val(:,2),STOREB_DminN05(f).val(:,6),ms(f),sh{2},'MarkerEdgeColor','black','MarkerFaceColor',colA(f,:)','MarkerFaceAlpha',alpha);    

    % Trace optset: N=4/5 lambda4
    % scatter(a3,STOREA_DminN04(f).val(:,2),STOREA_DminN04(f).val(:,5),ms(f),sh{1},'MarkerEdgeColor','black','MarkerFaceColor',colA(f,:)','MarkerFaceAlpha',alpha);    
    scatter(a3,STOREA_DminN05(f).val(:,2),STOREA_DminN05(f).val(:,5),ms(f),sh{2},'MarkerEdgeColor','black','MarkerFaceColor',colA(f,:)','MarkerFaceAlpha',alpha);    

    % lambda4 optset: N=4/5 lambda4
    % scatter(a4,STOREB_DminN04(f).val(:,2),STOREB_DminN04(f).val(:,5),ms(f),sh{1},'MarkerEdgeColor','black','MarkerFaceColor',colA(f,:)','MarkerFaceAlpha',alpha);    
    scatter(a4,STOREB_DminN05(f).val(:,2),STOREB_DminN05(f).val(:,5),ms(f),sh{2},'MarkerEdgeColor','black','MarkerFaceColor',colA(f,:)','MarkerFaceAlpha',alpha);    

    % Trace optset: N=4/5 Nconn
    % scatter(a5,STOREA_DminN04(f).val(:,2),STOREA_DminN04(f).val(:,7),ms(f),sh{1},'MarkerEdgeColor','black','MarkerFaceColor',colA(f,:)','MarkerFaceAlpha',alpha);    
    scatter(a5,STOREA_DminN05(f).val(:,2),STOREA_DminN05(f).val(:,7),ms(f),sh{2},'MarkerEdgeColor','black','MarkerFaceColor',colA(f,:)','MarkerFaceAlpha',alpha);    

    % lambda4 optset: N=4/5 lambda4
    % scatter(a6,STOREB_DminN04(f).val(:,2),STOREB_DminN04(f).val(:,7),ms(f),sh{1},'MarkerEdgeColor','black','MarkerFaceColor',colA(f,:)','MarkerFaceAlpha',alpha);    
    scatter(a6,STOREB_DminN05(f).val(:,2),STOREB_DminN05(f).val(:,7),ms(f),sh{2},'MarkerEdgeColor','black','MarkerFaceColor',colA(f,:)','MarkerFaceAlpha',alpha);    

end

% legend A
[l,icons,~,~]  = legend(a1,'tr Dmin: 1','tr Dmin: 2','tr Dmin: 3');
l.LineWidth = 2;
for i = 1:3
  icons(3+i).Children.MarkerSize = 12+4*(i-1);
end

% legebd B
[l,icons,~,~]  = legend(a2,'\lambda_4 Dmin: 1','\lambda_4 Dmin: 2','\lambda_4 Dmin: 3');
l.LineWidth = 2;
for i = 1:3
  icons(3+i).Children.MarkerSize = 12+4*(i-1);
end
end

%%
% set figure for lambda4/trace/Nconn
if 1
figure(2);

% set figure for Rho
a1 = subplot(2,2,1);
a1.LineWidth = 2;
hold on; grid on; box on;
xlim([1.5 8.5]); ylim([0 0.7]);
set(gca,'fontsize', 20);
ylabel('\rho^2');

a2 = subplot(2,2,2);
a2.LineWidth = 2;
hold on; grid on; box on;
xlim([1.5 8.5]); ylim([0 0.7]);
set(gca,'fontsize', 20);

% set figure for Lambda4
a3 = subplot(2,2,3);
a3.LineWidth = 2;
hold on; grid on; box on;
xlim([1.5 8.5]); ylim([0 80]);
set(gca,'fontsize', 20);
ylabel('\lambda_4'); xlabel('Dmax'); 

a4 = subplot(2,2,4);
a4.LineWidth = 2;
hold on; grid on; box on;
xlim([1.5 8.5]); ylim([0 80]);
set(gca,'fontsize', 20);


% first column - N=4
scatter(a1,STOREA_DminN04(2).val(:,2),STOREB_DminN04(2).val(:,4),ms(1),sh{1},'MarkerEdgeColor','black','MarkerFaceColor',colA(1,:)','MarkerFaceAlpha',alpha);    
scatter(a1,STOREC_DminN04(2).val(:,2),STOREC_DminN04(2).val(:,4),ms(1),sh{2},'MarkerEdgeColor','black','MarkerFaceColor',colA(2,:)','MarkerFaceAlpha',alpha);    
scatter(a1,STORED_DminN04(2).val(:,2),STORED_DminN04(2).val(:,4),ms(1),sh{3},'MarkerEdgeColor','black','MarkerFaceColor',colA(3,:)','MarkerFaceAlpha',alpha);    
       
scatter(a3,STOREA_DminN04(2).val(:,2),STOREB_DminN04(2).val(:,5),ms(1),sh{1},'MarkerEdgeColor','black','MarkerFaceColor',colA(1,:)','MarkerFaceAlpha',alpha);    
scatter(a3,STOREC_DminN04(2).val(:,2),STOREC_DminN04(2).val(:,5),ms(1),sh{2},'MarkerEdgeColor','black','MarkerFaceColor',colA(2,:)','MarkerFaceAlpha',alpha);    
scatter(a3,STORED_DminN04(2).val(:,2),STORED_DminN04(2).val(:,5),ms(1),sh{3},'MarkerEdgeColor','black','MarkerFaceColor',colA(3,:)','MarkerFaceAlpha',alpha);    

% first column - N=5
scatter(a2,STOREA_DminN05(2).val(:,2),STOREB_DminN05(2).val(:,4),ms(1),sh{1},'MarkerEdgeColor','black','MarkerFaceColor',colA(1,:)','MarkerFaceAlpha',alpha);    
scatter(a2,STOREC_DminN05(2).val(:,2),STOREC_DminN05(2).val(:,4),ms(1),sh{2},'MarkerEdgeColor','black','MarkerFaceColor',colA(2,:)','MarkerFaceAlpha',alpha);    
scatter(a2,STORED_DminN05(2).val(:,2),STORED_DminN05(2).val(:,4),ms(1),sh{3},'MarkerEdgeColor','black','MarkerFaceColor',colA(3,:)','MarkerFaceAlpha',alpha);    
       
scatter(a4,STOREA_DminN05(2).val(:,2),STOREB_DminN05(2).val(:,5),ms(1),sh{1},'MarkerEdgeColor','black','MarkerFaceColor',colA(1,:)','MarkerFaceAlpha',alpha);    
scatter(a4,STOREC_DminN05(2).val(:,2),STOREC_DminN05(2).val(:,5),ms(1),sh{2},'MarkerEdgeColor','black','MarkerFaceColor',colA(2,:)','MarkerFaceAlpha',alpha);    
scatter(a4,STORED_DminN05(2).val(:,2),STORED_DminN05(2).val(:,5),ms(1),sh{3},'MarkerEdgeColor','black','MarkerFaceColor',colA(3,:)','MarkerFaceAlpha',alpha);    


% legend A
[l,icons,~,~]  = legend(a1,'J = 1/\lambda_4','J = 1/\lambda_4+\rho^2','J = 1/\lambda_4 & |\rho|>0.5');
l.LineWidth = 2;
for i = 1:3
  icons(3+i).Children.MarkerSize = 12;
end
end
