
clc;
clear;
close all;

% op = 1;     % 1 predetermined; 2 active 
% op_trans = 2;   % 1 straight path; 2 circle path
% op_noise = 2;   % sigma_feat = 0.05; 2 sigma_feat is related to distance

load(['result/EKF121.mat'], 'er2', 'ef2', 't');
er2_122 = er2;
ef2_122 = ef2;
t_122 = t;

load(['result/EKF221.mat'], 'er2', 'ef2', 't');
er2_222 = er2;
ef2_222 = ef2;
t_222 = t;

load(['result/EKF1121.mat'], 'er2', 'ef2', 't');
er2_1122 = er2;
ef2_1122 = ef2;
t_1122 = t;

load(['result/EKF2121.mat'], 'er2', 'ef2', 't');
er2_2122 = er2;
ef2_2122 = ef2;
t_2122 = t;

w = 1.5;
fontsize = 14;
figure(1);
her_122 = plot(1:length(er2_122), er2_122, 'g', 'linewidth', w);
hold on;
her_222 = plot(1:length(er2_222), er2_222, 'y', 'linewidth', w);
her_1122 = plot(1:length(er2_1122), er2_1122, 'b', 'linewidth', w);
her_2122 = plot(1:length(er2_2122), er2_2122, 'm', 'linewidth', w);
legend('predetermined', 'local', 'global', 'combined');
set(gca,'FontSize',fontsize);
        xlabel('X(step)');
        ylabel('Y(cm)'); 
savefig('fig/er1.fig');

figure(2);
hef_122 = plot(1:length(ef2_122), ef2_122, 'g', 'linewidth', w);
hold on;
hef_222 = plot(1:length(ef2_222), ef2_222, 'y', 'linewidth', w);
hef_1122 = plot(1:length(ef2_1122), ef2_1122, 'b', 'linewidth', w);
hef_2122 = plot(1:length(ef2_2122), ef2_2122, 'm', 'linewidth', w);
legend('predetermined', 'local', 'global', 'combined');
set(gca,'FontSize',fontsize); 
        xlabel('X(step)');
        ylabel('Y(cm)');
savefig('fig/ef1.fig');

figure(3);
ht_122 = plot(1:length(t_122), t_122, 'g', 'linewidth', w);
hold on;
ht_222 = plot(1:length(t_222), t_222, 'y', 'linewidth', w);
ht_1122 = plot(1:length(t_1122), t_1122, 'b', 'linewidth', w);
ht_2122 = plot(1:length(t_2122), t_2122, 'm', 'linewidth', w);
legend('predetermined', 'local', 'global', 'combined');
set(gca,'FontSize',fontsize); 
        xlabel('X(step)');
        ylabel('Y(s)');
savefig('fig/t1.fig');