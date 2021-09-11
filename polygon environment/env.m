%%
clc;
clear;
close all;

%% Initialization
op_trans = 2;   % 1 straight path; 2 circle path

record = 1;
if record == 1
    video = VideoWriter(['video/env.avi']);
    video.FrameRate = 5;
    video.Quality = 100;
    open(video);
end

load dataFeature M feature translation deform
load dataFeature id1 id2 tri1 tri2
N = 30;
N_elipse = 201;

Pose = [0; 0; 0; 0; 0; 0];
% Pose = [0; 0; 0; -0.5; 0; 0];
rotation_t = eye(3);
rotation_d = eye(3);
range = [0.1;3];

load sigma sigma_head sigma_odem sigma_level sigma_smoo sigma_cons sigma_deform
load noise/noise2 noise_odem noise_cons noise_smoo noise_deform
%%
for i = 1:N
    i
    f = figure(1);
    edge = [];
    edge1 = [];
    i_trans = mod(i, 6);
    if i_trans == 0 
        i_trans = 6;
    end
    for j = 1:M
        feature_frame{j}(1:3, i) = feature{i}(1:3, j);
        feature{i + 1}(1:3, j) = feature{i}(1:3, j) + rotation_t * translation(1:3, i_trans) + noise_smoo{j, i};
        d_local_feature{j}(1:3, i) = deform{j}(1:3, i_trans);
        feature_frame{j}(1:3, i) = feature_frame{j}(1:3, i) + rotation_d * d_local_feature{j}(1:3, i) + noise_deform{j, i};
        local_cordinate_feature{j}(1:3, i) = feature_frame{j}(1:3, i) - feature_frame{1}(1:3, i);
        feature_i(:, j) = feature_frame{j}(1:3, i);
        edge1 = [edge1, feature{i}(1:3, j)];
        edge = [edge, feature_frame{j}(1:3, i)];
    end
    
    if i == 1
        %plot
        hfg = plot3(feature_i(1, :), feature_i(2, :), feature_i(3, :), 'p', 'color', [0 176 80]/250, 'MarkerFaceColor', [0 176 80]/250, 'MarkerSize', 15);
        hold on;
        grid on;
        
        hedg_old = plot3(edge1(1, :), edge1(2, :), edge1(3, :), '--', 'color', [150 150 150]/250, 'linewidth',1);
        hedg = plot3(edge(1, :), edge(2, :), edge(3, :), '--k', 'linewidth',1);
       
        set(gca,'FontSize',28);
        xlabel('X(cm)');
        ylabel('Y(cm)');
        zlabel('Z(cm)');
%         if op_trans == 1
            axis([0 3 -2 3 0 5]);
%         else
%             axis([-10 10 -2 16 -1 25]);
%         end
        set(gcf,'unit','normalized','position',[0, 0, 0.65, 1], 'color', 'w');
    else
        figure(1)
            set(hfg, 'XData', feature_i(1, :), 'YData', feature_i(2, :), 'ZData', feature_i(3, :));
            set(hedg, 'XData', edge(1, :), 'YData', edge(2, :), 'ZData', edge(3, :));
    end
    if record == 1
        F(i) = getframe(f);
        writeVideo(video, F(i));
    end
end
if record == 1
    close(video);
%     savefig(['fig/EKF' num2str(op) num2str(op_trans) num2str(op_noise) '.fig']);
end