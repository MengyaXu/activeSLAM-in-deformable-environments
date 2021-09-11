%%
clc;
clear;
close all;

%% Initialization
op = 2;     % 1 predetermined; 2 active 
op_trans = 2;   % 1 straight path; 2 circle path
op_noise = 1;   % sigma_feat = 0.05; 2 sigma_feat is related to distance

op_newNoise = 0; % 0 existing noise_feat; 1 new generated noise_feat
record_noise = 0;

record = 1;
if record == 1
    video = VideoWriter(['video/' num2str(op) num2str(op_trans) num2str(op_noise) '.avi']);
    video.FrameRate = 5;
    video.Quality = 100;
    open(video);
end

load dataFeature M feature translation deform M_all feature_all
load dataFeature id1 id2 tri1 tri2
N = 200;
N_elipse = 201;

Pose = [0; 0; 0; 0; 0; 0];
% Pose = [0; 0; 0; -0.5; 0; 0];
rotation_t = eye(3);
rotation_d = eye(3);
range = [0.1;10];

load sigma sigma_head sigma_odem sigma_smoo sigma_cons sigma_deform
load noise/noise2 noise_odem noise_cons noise_smoo noise_deform
if op_noise == 1
%     load sigma sigma_feat
    sigma_r_ = 0.03;
    sigma_b_ = 0.03;
    load noise/noise2 noise_feat
end

if op_noise == 2
    sigma_r0 = 0.01;
    sigma_r1 = 0.09;
    dr_sigma = (sigma_r1 - sigma_r0) / (range(2) - range(1));
    
    sigma_b0 = 0.01;
    sigma_b1 = 0.09;
    db_sigma = (sigma_b1 - sigma_b0) / (range(2) - range(1));
end
save data M op range op_noise op_trans
% visble(:, 1) = ones(3, 1);
num_unseen = 0;
num_vis = 0;
vis_feat = [];
constraint_edge = [];
% % n = 0;
% for i = 1:M
%     for j = i+1:M
%         constraint_edge = [constraint_edge; i, j];
% %         n = n + 1;
%     end
% end
N_cons = sparse(sigma_cons^2*eye(3*length(constraint_edge)));
save data constraint_edge N_cons -append
%%
er2 = [];
ef2 = [];
erP2 = 0;
efP2 = 0;
t = [];
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
%         edge1 = [edge1, feature{i}(1:3, j)];
%         edge = [edge, feature_frame{j}(1:3, i)];
    end
    for j = 1:M
        feature_frame{j}(1:3, i) = feature{i}(1:3, j);
        feature{i + 1}(1:3, j) = feature{i}(1:3, j) + rotation_t * translation(1:3, i_trans) + noise_smoo{j, i};
        d_local_feature{j}(1:3, i) = deform{ceil(j/M*M_all)}(1:3, i_trans);
        feature_frame{j}(1:3, i) = feature_frame{j}(1:3, i) + rotation_d * d_local_feature{j}(1:3, i) + noise_deform{j, i};
        local_cordinate_feature{j}(1:3, i) = feature_frame{j}(1:3, i) - feature_frame{1}(1:3, i);
        feature_i(:, j) = feature_frame{j}(1:3, i);
%         edge1 = [edge1, feature{i}(1:3, j)];
%         edge = [edge, feature_frame{j}(1:3, i)];
    end
    
    sigma_Feat{i} = [];
    vis_line = [];
    unvis_line = [];
    z = [];
    z_est = [];
    if i == 1
        x = Pose;
        for j = 1:M
            y = (R(Pose(3),'y',0)*R(Pose(2),'p',0)*R(Pose(1),'r',0))' * (feature_frame{j}(1:3, i) - Pose(4:6, i));
            zj = y;
            q = sqrt(zj' * zj);
            if op_noise == 2
                sigma_r{j, i} = sigma_r0 + dr_sigma * max(0, q - range(1));
                sigma_b{j, i} = sigma_b0 + db_sigma * max(0, q - range(1));
                if op_newNoise == 1
                    noise_feat{j, i} = [normrnd(0, sigma_r{j, i}, 2, 1); normrnd(0, sigma_b{j, i})];
                    while noise_feat{j, i}(1) > 2*sigma_r{j, i} || noise_feat{j, i}(1) < -2*sigma_r{j, i}
                        noise_feat{j, i}(1) = normrnd(0, sigma_r{j, i}, 1, 1);
                    end
                    while noise_feat{j, i}(2) > 2*sigma_r{j, i} || noise_feat{j, i}(2) < -2*sigma_r{j, i}
                        noise_feat{j, i}(2) = normrnd(0, sigma_r{j, i}, 1, 1);
                    end
                    while noise_feat{j, i}(3) > 2*sigma_b{j, i} || noise_feat{j, i}(3) < -2*sigma_b{j, i}
                        noise_feat{j, i}(3) = normrnd(0, sigma_b{j, i}, 1, 1);
                    end 
                else
                    load(['noise/noiseEKF' num2str(op) num2str(op_trans) num2str(op_noise) '_' num2str(N) '.mat'], 'noise_feat');
%                     load(['noise/noiseEKF211.mat'], 'noise_feat');
                end
            else
                sigma_r{j, i} = sigma_r_;
                sigma_b{j, i} = sigma_b_;
            end
            zj = zj + noise_feat{j, i};
            z = [z; zj];
            sigma_feat2 = [sigma_r{j, i}^2; sigma_r{j, i}^2; sigma_b{j, i}^2];
            sigma_Feat{i} = [sigma_Feat{i}; sigma_feat2];
            
            xfj = x(4:6) + zj;
            x = [x; xfj];
            if q <= range(2) && q >= range(1) && y(1) > 0
                vis = visibility(feature_i, Pose(4:6, i), j);
                if vis == 1
                    visble(j, i) = 1;
                    vis_line = [vis_line, Pose(4:6, i), feature_i(:, j)];
%                     for k = 1:length(vis_feat)
%                         constraint_edge = [constraint_edge; j, vis_feat(k)];
%                     end
                    vis_feat = [vis_feat; j];
%                     save data constraint_edge vis_feat -append
                else
                    visble(j, i) = 0;
                    unvis_line = [unvis_line, Pose(4:6, i), feature_i(:, j)];
                end
            else
                visble(j, i) = 0;
%                 unvis_line = [unvis_line, Pose(4:6, i), feature_i(:, j)];
            end
        end
        P = zeros(6 + 3*M, 6 + 3*M);
        P(7:end,7:end) = diag(sigma_Feat{i});
        P = sparse(P);
        X = x;
        x_pre = x;
        xr = X(4:6);
        traceP_all = trace(P);
        
        %plot
%         for j = 1:M
%             plot(feature_frame{j}(1, i), feature_frame{j}(2, i), 'p', 'MarkerEdgeColor', [0 j/M j/M], 'MarkerFaceColor', [0 j/M j/M], 'MarkerSize', 10);
%             hold on;
%             plot(x(3 + 2 * j - 1), x(3 + 2 * j), '*', 'Color', [j/M, 0, j/M]);
%         end

%         for k = 1:length(tri1(:, 1))
%             for j1 = 1:3
%                 id = id1(tri1(k, j1));
%                 f(:, j1) = feature_i(:, id);
%             end
%             hedge1 = plot3(f(1, :), f(2, :), f(3, :), 'k--');
%             hold on;
%         end
%         for k = 1:length(tri2(:, 1))
%             for j2 = 1:3
%                 id = id2(tri2(k, j2));
%                 f(:, j2) = feature_i(:, id);
%             end
%             hedge2 = plot3(f(1, :), f(2, :), f(3, :), 'k--');
%             hold on;
%         end
        hfg = plot3(feature_i(1, :), feature_i(2, :), feature_i(3, :), 'p', 'color', [0 176 80]/250, 'MarkerFaceColor', [0 176 80]/250, 'MarkerSize', 15);
        hold on;
        grid on;
        hfall = plot3(f_all_i(1, :), f_all_i(2, :), f_all_i(3, :), '.', 'color', [150 150 150]/250, 'MarkerSize', 5);
%         hedg_old = plot3(edge1(1, :), edge1(2, :), edge1(3, :), '--', 'color', [150 150 150]/250, 'linewidth',1);
%         hedg = plot3(edge(1, :), edge(2, :), edge(3, :), '--k', 'linewidth',1);
        hrg = plot3(Pose(4, :), Pose(5, :), Pose(6, :), 'k+-', 'MarkerSize', 10);
        hold on;

        if ~isempty(vis_line)
            hvis = plot3(vis_line(1, :), vis_line(2, :), vis_line(3, :),'-g','linewidth',1.5);
        end
        if ~isempty(unvis_line)
            hunvis = plot3(unvis_line(1, :), unvis_line(2, :), unvis_line(3, :),'--y','linewidth',1.5);
        else
            hunvis = plot3(0, 0, 0, '--y','linewidth',1.5);
        end

        hr = plot3(xr(1, :), xr(2, :), xr(3, :), 'b.-', 'MarkerSize', 10);
        rexy = plot_elipse(xr(1:2), P(2:3, 2:3));
        hrexy = plot3(rexy(1, :), rexy(2, :), xr(3, i)*ones(1, N_elipse), '.b', 'MarkerSize',0.5);
        rexz = plot_elipse(xr(1:2:3), P(2:2:4, 2:2:4));
        hrexz = plot3(rexz(1, :), xr(2, i)*ones(1, N_elipse), rexz(2, :), '.b', 'MarkerSize',0.5);
        rezy = plot_elipse(xr(2:3), P(3:4, 3:4));
        hrezy = plot3(xr(1, i)*ones(1, N_elipse), rezy(1, :), rezy(2, :), '.b', 'MarkerSize',0.5);

        hrp = plot3(x_pre(4), x_pre(5), x_pre(6), 'mx');
        hfp = plot3(x_pre(7:3:end), x_pre(8:3:end), x_pre(9:3:end), 'mx');
        hf = plot3(x(7:3:end), x(8:3:end), x(9:3:end), 'r*', 'MarkerSize', 10);
        hold on;
        
        r=R(x(3),'y',0)*R(x(2),'p',0)*R(x(1),'r',0);
        axis_x = r'*[1; 0; 0]*1;
        axis_y = r'*[0; 1; 0]*1;
        axis_z = r'*[0; 0; 1]*1;
        hroll = plot3([x(4) axis_x(1)], [x(5) axis_x(2)], [x(6) axis_x(3)], 'r-');
        hpitch = plot3([x(4) axis_y(1)], [x(5) axis_y(2)], [x(6) axis_y(3)], 'm-');
        hyell = plot3([x(4) axis_z(1)], [x(5) axis_z(2)], [x(6) axis_z(3)], 'c-');

        fexy = [];
        fexz = [];
        fezy = [];
        xy = [];
        xz = [];
        yz = [];
        for j = 1:M
            fejxy = plot_elipse(feature_frame{j}(1:2, i), P(4+3*j-2:4+3*j-1, 4+3*j-2:4+3*j-1));
            fexy = [fexy, fejxy];
            xy = [xy, feature_frame{j}(3, i)*ones(1, N_elipse)];
            fejxz = plot_elipse(feature_frame{j}(1:2:3, i), P(4+3*j-2:2:4+3*j, 4+3*j-2:2:4+3*j));
            fexz = [fexz, fejxz];
            xz = [xz, feature_frame{j}(2, i)*ones(1, N_elipse)];
            fejzy = plot_elipse(feature_frame{j}(2:3, i), P(4+3*j-1:4+3*j, 4+3*j-1:4+3*j));
            fezy = [fezy, fejzy];
            yz = [yz, feature_frame{j}(1, i)*ones(1, N_elipse)];
        end
        hfexy = plot3(fexy(1, :), fexy(2, :), xy, '.k', 'MarkerSize',0.5);
        hfexz = plot3(fexz(1, :), xz, fexz(2, :), '.k', 'MarkerSize',0.5);
        hfezy = plot3(yz, fezy(1, :), fezy(2, :), '.k', 'MarkerSize',0.5);
        
        set(gca,'FontSize',20);
%         if op_trans == 1
            axis([-5 25 -15 15 -5 25]);
%         else
%             axis([-10 10 -2 16 -1 25]);
%         end
        set(gcf,'unit','normalized','position',[0, 0, 0.65, 1], 'color', 'w');
        save data visble -append
    else
        % smooth measurement
        measurement_smooth = [];
        i_trans = mod(i-1, 6);
        if i_trans == 0 
            i_trans = 6;
        end
        for j = 1:M
            measurement_smooth = [measurement_smooth; rotation_t * translation(1:3, i_trans) + d_local_feature{j}(1:3, i)-d_local_feature{j}(1:3, i-1)];
%             measurement_smooth = [measurement_smooth; d_local_feature{j}(1:3, i)-d_local_feature{j}(1:3, i-1)];
        end
        Q = sparse(blkdiag(sigma_head^2 * eye(3), sigma_odem^2 * eye(3), (sigma_smoo^2 + sigma_deform^2) * eye(3*M)));
%         Q = blkdiag(0^2*eye(2), sigma_head^2, sigma_odem^2 * eye(3), (sigma_smoo^2 + sigma_cons^2) * eye(3*M));
        
        % prediction
        save data Q P x measurement_smooth i -append
%         figure(i+1)
%         hfg = plot3(feature_i(1, :), feature_i(2, :), feature_i(3, :), 'p', 'color', [0 176 80]/250, 'MarkerFaceColor', [0 176 80]/250, 'MarkerSize', 15);
%         hold on;
%         grid on;
        
        tic;
        u(:, i - 1) = getV_trace(op, x);
        toc;
        tc = toc;
        t = [t; tc];
        
        [~, Pose(:, i)] = predict(1, u(:,i-1), Q, P, Pose(:, i-1), measurement_smooth, M, noise_odem(:, i));
%         [~, Pose(:, i)] = predict(1, u(:,i-1), Q, P, Pose(:, i-1), measurement_smooth, M, zeros(6, 1));
        [P_pre, x_pre] = predict(2, u(:,i-1), Q, P, x, measurement_smooth, M, noise_odem(:, i));
        %feature measurement
        for j = 1:M
            y = (R(Pose(3, i),'y',0)*R(Pose(2, i),'p',0)*R(Pose(1, i),'r',0))' * (feature_frame{j}(1:3, i) - Pose(4:6, i));
            zj = y;
            q = sqrt(zj' * zj);
            if q <= range(2) && q >= range(1) && y(1) > 0
                vis = visibility(feature_i, Pose(4:6, i), j);
                if vis == 1
                    if ~ismember(j, vis_feat)
%                         for k = 1:length(vis_feat)
%                             constraint_edge = [constraint_edge; j, vis_feat(k)];
%                         end
                        vis_feat = [vis_feat; j];
%                         save data constraint_edge vis_feat -append
                        if length(vis_feat) == M
                            step_vis = i;
                        end
                    end
                    if op_noise == 2
                        sigma_r{j, i} = sigma_r0 + dr_sigma * max(0, q - range(1));
                        sigma_b{j, i} = sigma_b0 + db_sigma * max(0, q - range(1));
                        if op_newNoise == 1
                            noise_feat{j, i} = [normrnd(0, sigma_r{j, i}, 2, 1); normrnd(0, sigma_b{j, i})];
                            while noise_feat{j, i}(1) > 2*sigma_r{j, i} || noise_feat{j, i}(1) < -2*sigma_r{j, i}
                                noise_feat{j, i}(1) = normrnd(0, sigma_r{j, i}, 1, 1);
                            end
                            while noise_feat{j, i}(2) > 2*sigma_r{j, i} || noise_feat{j, i}(2) < -2*sigma_r{j, i}
                                noise_feat{j, i}(2) = normrnd(0, sigma_r{j, i}, 1, 1);
                            end
                            while noise_feat{j, i}(3) > 2*sigma_b{j, i} || noise_feat{j, i}(3) < -2*sigma_b{j, i}
                                noise_feat{j, i}(3) = normrnd(0, sigma_b{j, i}, 1, 1);
                            end 
                        else
                            load(['noise/noiseEKF' num2str(op) num2str(op_trans) num2str(op_noise) '_' num2str(N) '.mat'], 'noise_feat');
    %                         load(['noise/noiseEKF212.mat'], 'noise_feat');
                        end
                    else
                        sigma_r{j, i} = sigma_r_;
                        sigma_b{j, i} = sigma_b_;
                    end
                    zj = zj + noise_feat{j, i};
                    z = [z; zj];
                    sigma_feat2 = [sigma_r{j, i}^2; sigma_r{j, i}^2; sigma_b{j, i}^2];
                    sigma_Feat{i} = [sigma_Feat{i}; sigma_feat2];

                    visble(j, i) = 1;
                    vis_line = [vis_line, Pose(4:6, i), feature_i(:, j)];

                    rot = R(x_pre(3),'y',0)*R(x_pre(2),'p',0)*R(x_pre(1),'r',0);
                    delta = x_pre(6+3*j-2:6+3*j) - x_pre(4:6);
                    y = rot' * delta;
                    z_est = [z_est; y];

                    Rr = R(x_pre(3),'y',0)*R(x_pre(2),'p',0)*R(x_pre(1),'r',1);
                    Rp = R(x_pre(3),'y',0)*R(x_pre(2),'p',1)*R(x_pre(1),'r',0);
                    Ry = R(x_pre(3),'y',1)*R(x_pre(2),'p',0)*R(x_pre(1),'r',0);
                    Hij = [Rr'*delta Rp'*delta Ry'*delta, -rot', rot'];
                    H_feat{i, j} = Hij;
                    
                    num_vis = num_vis + 1;
                else
                    visble(j, i) = 0;
                    unvis_line = [unvis_line, Pose(4:6, i), feature_i(:, j)];
                    num_unseen = num_unseen + 1;
                end
            else
                visble(j, i) = 0;
%                 unvis_line = [unvis_line, Pose(4:6, i), feature_i(:, j)];
                num_unseen = num_unseen + 1;
            end
        end
        % construction measurement
        if size(constraint_edge, 1) ~= 0
            for j = 1:size(constraint_edge, 1)
                measurement_construction{i, j} = local_cordinate_feature{constraint_edge(j, 2)}(1:3, i) ...
                    - local_cordinate_feature{constraint_edge(j,1)}(1:3,i) + noise_cons{ceil(constraint_edge(j, 1)/length(noise_cons(1,:))), constraint_edge(j, 2)};
            end
        else
            measurement_construction{i, j} = [];
        end
%         %KF Gain
%         N_cons = sparse(sigma_cons^2*eye(3*length(constraint_edge)));
        N_ = blkdiag(sparse(diag(sigma_Feat{i})), N_cons);
        [K, H] = KF_gain(P_pre, N_, visble(:, i), constraint_edge, M, H_feat, i);
        
        % update
        [x, P] = KF_update(x_pre, K, measurement_construction, z, H, P_pre, i, constraint_edge, z_est);
        
%         [x, P]=KF_gain_update(P_pre, N_, visble(:, i), constraint_edge, M, H_feat, i, ...
%     z, z_est, measurement_construction, x_pre);
        
        X = [X, x];
        xr = [xr, x(4:6)];
        traceP_all = traceP_all + trace(P);
        
        % plot
            %plot
            
%             for k = 1:length(tri1(:, 1))
%                 for j1 = 1:3
%                     id = id1(tri1(k, j1));
%                     f(:, j1) = feature_i(:, id);
%                 end
%                 set(hedge1, 'XData', f(1, :), 'YData', f(2, :), 'ZData', f(3, :));
%             end
%             for k = 1:length(tri2(:, 1))
%                 for j2 = 1:3
%                     id = id2(tri2(k, j2));
%                     f(:, j2) = feature_i(:, id);
%                 end
%                 set(hedge2, 'XData', f(1, :), 'YData', f(2, :), 'ZData', f(3, :));
%             end
            
        
        figure(1)
            set(hfg, 'XData', feature_i(1, :), 'YData', feature_i(2, :), 'ZData', feature_i(3, :));
            set(hfall, 'XData', f_all_i(1, :), 'YData', f_all_i(2, :), 'ZData', f_all_i(3, :));
%             hedg_old = plot3(edge1(1, :), edge1(2, :), edge1(3, :), '--', 'color', [100 100 100]/250, 'linewidth',1);
%             set(hedg, 'XData', edge(1, :), 'YData', edge(2, :), 'ZData', edge(3, :));
%             edge1 = edge;
            
            set(hrg, 'XData', Pose(4, :), 'YData', Pose(5, :), 'ZData', Pose(6, :));

            if ~isempty(vis_line)
                set(hvis, 'XData', vis_line(1, :), 'YData', vis_line(2, :), 'ZData', vis_line(3, :));
            else
                set(hvis, 'XData', 0, 'YData', 0, 'ZData', 0);            
            end
            if ~isempty(unvis_line)
                set(hunvis, 'XData', unvis_line(1, :), 'YData', unvis_line(2, :), 'ZData', unvis_line(3, :));
            else
                set(hunvis, 'XData', 0, 'YData', 0, 'ZData', 0);     
            end

            set(hr, 'XData', xr(1, :), 'YData', xr(2, :), 'ZData', xr(3, :));
            rexy = plot_elipse(xr(1:2, i), P(4:5, 4:5));
            set(hrexy, 'XData', rexy(1, :), 'YData', rexy(2, :), 'ZData', xr(3, i)*ones(1, N_elipse));
            rezy = plot_elipse(xr(1:2:3, i), P(4:2:6, 4:2:6));
            set(hrezy, 'XData', rezy(1, :), 'YData', xr(2, i)*ones(1, N_elipse), 'ZData', rezy(2, :));
            rexz = plot_elipse(xr(2:3, i), P(5:6, 5:6));
            set(hrexz, 'XData', xr(1, i)*ones(1, N_elipse), 'YData', rexz(1, :), 'ZData', rexz(2, :));            

            r=R(x(3),'y',0)*R(x(2),'p',0)*R(x(1),'r',0);
            axis_x = x(4:6) + r*[1; 0; 0]*1;
            axis_y = x(4:6) + r*[0; 1; 0]*1;
            axis_z = x(4:6) + r*[0; 0; 1]*1;
        hroll = plot3([x(4) axis_x(1)], [x(5) axis_x(2)], [x(6) axis_x(3)], 'r-');
        hpitch = plot3([x(4) axis_y(1)], [x(5) axis_y(2)], [x(6) axis_y(3)], 'm-');
        hyell = plot3([x(4) axis_z(1)], [x(5) axis_z(2)], [x(6) axis_z(3)], 'c-');
%             set(hroll, 'XData', [x(4) axis_x(1)], 'YData', [x(5) axis_x(2)], 'ZData', [x(6) axis_x(3)]);
%             set(hpitch, 'XData', [x(4) axis_y(1)], 'YData', [x(5) axis_y(2)], 'ZData', [x(6) axis_y(3)]);
%             set(hyell, 'XData', [x(4) axis_z(1)], 'YData', [x(5) axis_z(2)], 'ZData', [x(6) axis_z(3)]);
        
            set(hrp, 'XData', x_pre(4), 'YData', x_pre(5), 'ZData', x_pre(6));
            set(hfp, 'XData', x_pre(7:3:end), 'YData', x_pre(8:3:end), 'ZData', x_pre(9:3:end));
            set(hf, 'XData', x(7:3:end), 'YData', x(8:3:end), 'ZData', x(9:3:end));
            fexy = [];
            fexz = [];
            fezy = [];
            xy = [];
            xz = [];
            yz = [];            
            for j = 1:M
                fejxy = plot_elipse(feature_frame{j}(1:2, i), P(6+3*j-2:6+3*j-1, 6+3*j-2:6+3*j-1));
                fexy = [fexy, fejxy];
                xy = [xy, feature_frame{j}(3, i)*ones(1, N_elipse)];
                fejxz = plot_elipse(feature_frame{j}(1:2:3, i), P(6+3*j-2:2:6+3*j, 6+3*j-2:2:6+3*j));
                fexz = [fexz, fejxz];
                xz = [xz, feature_frame{j}(2, i)*ones(1, N_elipse)];
                fejzy = plot_elipse(feature_frame{j}(2:3, i), P(6+3*j-1:6+3*j, 6+3*j-1:6+3*j));
                fezy = [fezy, fejzy];
                yz = [yz, feature_frame{j}(1, i)*ones(1, N_elipse)];
            end
            set(hfexy, 'XData', fexy(1, :), 'YData', fexy(2, :), 'ZData', xy);
            set(hfexz, 'XData', fexz(1, :), 'YData', xz, 'ZData', fexz(2, :));
            set(hfezy, 'XData', yz, 'YData', fezy(1, :), 'ZData', fezy(2, :));
    end
    %     axis([0 5 0 15.5]);
    %     pause(0.5);
    if record == 1
        F(i) = getframe(f);
        writeVideo(video, F(i));
    end
    er = x(1:6) - Pose(:, i);
    er(1) = wrapToPi(er(1));
    er(2) = wrapToPi(er(2));
    er(3) = wrapToPi(er(3));
    er2 = [er2; sqrt(er' * er)];
    if i ~= 1
        erP2 = erP2 + er' * P(1:6, 1:6)^(-1) * er;
    end
    if i == N
        for j = 1:M
            ef = x(6+3*j-2:6+3*j) - feature_frame{j}(1:3, i);
            ef2 = [ef2; sqrt(ef' * ef)];
            efP2 = efP2 + ef' * P(6+3*j-2:6+3*j, 6+3*j-2:6+3*j)^(-1) * ef;
        end
    end
end
if op_newNoise == 1 && record_noise == 1
    save(['noise/noiseEKF' num2str(op) num2str(op_trans) num2str(op_noise) '_' num2str(N) '.mat'], 'noise_feat');
end
u
traceP = trace(P)
num_vis
step_vis
er_max = max(er2);
er_ave = sum(er2)/length(er2);
ef_max = max(ef2);
ef_ave = sum(ef2) / length(ef2);
if record == 1
    close(video);
    save(['result/EKF' num2str(op) num2str(op_trans) num2str(op_noise) '.mat'], 'u', 'traceP', 'num_unseen',...
        'er2', 'ef2', 'erP2', 'efP2', 'traceP_all', 'step_vis', 'num_vis', 't', ...
        'er_max', 'er_ave', 'ef_max', 'ef_ave');
    savefig(['fig/EKF' num2str(op) num2str(op_trans) num2str(op_noise) '.fig']);
end