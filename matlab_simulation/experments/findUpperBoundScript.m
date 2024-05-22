% Example data
x =cov_det_list;
y =lambda4;

p = findUpperBoundFunction(x, y);

% Plot the original data
% plot(x, y, 'o');
% hold on;

% Plot the final fitted line
xFit = x;
yFit = polyval(p, xFit);
figure;
plot(xFit, yFit,"LineStyle","-","Color","#7E2F8E", 'LineWidth', 1); hold on
plot(x, y, "LineStyle","none","Marker",'+',"Color","#A2142F");
xlabel('$\sqrt{\det{cov[X]}}$','Interpreter','latex');
ylabel('\lambda_4');
grid on; box on;
% title('Upper Boundary Line Fit');


hold off;
%%
clear all; clc; close all;
data = load("results\m10to50.mat");
%%
log = data.log;
m_list = data.p_list(:,1);
exp_list = [3,7,14,21];
p_list = data.p_list;
figure;
for i = 1:length(exp_list)
    exp_num = exp_list(i);
    m = m_list(exp_num);
    p = p_list(exp_num,2:3);
    eig_list = log{exp_num,1};
    lambda4 = eig_list(:,4);
    cov_dat = log{exp_num,3};
    subplot(2,2,i); 
    lambda4_fit = polyval(p, cov_dat);
    plot(cov_dat, lambda4_fit,"LineStyle","-","Color","#7E2F8E", 'LineWidth', 1); hold on
    plot(cov_dat, lambda4, "LineStyle","none","Marker",'+',"Color","#A2142F");
    % xlabel('$\sqrt{\det{cov[X]}}$','Interpreter','latex');
    % ylabel('\lambda_4');
    title(['m = ',num2str(m)]);
    grid on; box on;
end
subplot(2,2,1); ylabel('$\lambda_4$','Interpreter','latex');
subplot(2,2,3); ylabel('$\lambda_4$','Interpreter','latex'); xlabel('$\sqrt{\det{cov[X]}}$','Interpreter','latex');
subplot(2,2,4); xlabel('$\sqrt{\det{cov[X]}}$','Interpreter','latex');
% plot(cov_dat, lambda4, "LineStyle","none","Marker",'+',"Color","#A2142F");

%%
figure;plot(m_list,p_list(:,2), "LineStyle","none","Marker",'+',"Color","#A2142F"); hold on;
pa = polyfit(m_list,p_list(:,2),1);
afit = polyval(pa,m_list); 
plot(m_list, afit, "LineStyle","-","Color","#7E2F8E", 'LineWidth', 1);grid on; box on;
ylabel('slope','Interpreter','latex','FontSize',15);
xlabel('$m$','Interpreter','latex','FontSize',15);
xl = xlim;
yl = ylim;
xt = 0.05 * (xl(2)-xl(1)) + xl(1);
yt = 0.90 * (yl(2)-yl(1)) + yl(1);
caption = sprintf('slope = %f * m + %f', pa(1), pa(2));
text(xt, yt, caption, 'FontSize', 12,'Color', "k");