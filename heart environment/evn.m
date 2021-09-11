%%
clc;
clear;
close all;

%% Initialization
record_noise = 1;

record = 1;
if record == 1
    video = VideoWriter(['video/env_heart_.avi']);
    video.FrameRate = 5;
    video.Quality = 100;
    open(video);
end

load dataFeature M feature translation deform M_all feature_all
load dataFeature id1 id2 tri1 tri2
N = 30;

Pose = [0; 0; 0; 0; 0; 0];
% Pose = [0; 0; 0; -0.5; 0; 0];
rotation_t = eye(3);
rotation_d = eye(3);

load sigma sigma_head sigma_odem sigma_smoo sigma_cons sigma_deform
load noise/noise2 noise_odem noise_cons noise_smoo noise_deform

for i = 1:N
    i
    f = figure(1);
%     edge = [];
%     edge1 = [];
    i_trans = mod(i, 6);
    if i_trans == 0 
        i_trans = 6;
    end
    for j = 1:M_all
        jid = ceil(j/M_all*M);
        f_all_frame{j}(1:3, i) = feature_all{i}(1:3, j);
        feature_all{i + 1}(1:3, j) = feature_all{i}(1:3, j) + rotation_t * translation(1:3, i_trans) + noise_smoo{jid, i};
        d_local_all{j}(1:3, i) = deform{j}(1:3, i_trans);
        f_all_frame{j}(1:3, i) = f_all_frame{j}(1:3, i) + rotation_d * d_local_all{j}(1:3, i) + noise_deform{jid, i};
        f_all_i(:, j) = f_all_frame{j}(1:3, i);
    end
    for j = 1:M
        feature_frame{j}(1:3, i) = feature{i}(1:3, j);
        feature{i + 1}(1:3, j) = feature{i}(1:3, j) + rotation_t * translation(1:3, i_trans) + noise_smoo{j, i};
        d_local_feature{j}(1:3, i) = deform{ceil(j/M*M_all)}(1:3, i_trans);
        feature_frame{j}(1:3, i) = feature_frame{j}(1:3, i) + rotation_d * d_local_feature{j}(1:3, i) + noise_deform{j, i};
        local_cordinate_feature{j}(1:3, i) = feature_frame{j}(1:3, i) - feature_frame{1}(1:3, i);
        feature_i(:, j) = feature_frame{j}(1:3, i);
    end
    
    if i == 1
        %plot
        hfg = plot3(feature_i(1, :), feature_i(2, :), feature_i(3, :), 'p', 'color', [0 176 80]/250, 'MarkerFaceColor', [0 176 80]/250, 'MarkerSize', 15);
        hold on;
        grid on;
        hfall = plot3(f_all_i(1, :), f_all_i(2, :), f_all_i(3, :), '.', 'color', [150 150 150]/250, 'MarkerSize', 5);
        hold on;
        grid on;
        
        xlabel('X(cm)');
        ylabel('Y(cm)');
        zlabel('Z(cm)');
        set(gca,'FontSize',28);
            axis([-5 25 -15 15 -5 25]);
        set(gcf,'unit','normalized','position',[0, 0, 0.65, 1], 'color', 'w');
    else
        set(hfg, 'XData', feature_i(1, :), 'YData', feature_i(2, :), 'ZData', feature_i(3, :));
        set(hfall, 'XData', f_all_i(1, :), 'YData', f_all_i(2, :), 'ZData', f_all_i(3, :));

    end
    if record == 1
        F(i) = getframe(f);
        writeVideo(video, F(i));
    end
end
if record == 1
    close(video);
    savefig(['fig/env_heart_.fig']);
end