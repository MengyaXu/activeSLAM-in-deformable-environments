function obj = objFun2(u0)
load data op_noise range Q P x measurement_smooth i visble constraint_edge M N_cons
% load sigma sigma_cons

r=R(u0(3),'y',0)*R(u0(2),'p',0)*R(u0(1),'r',0);
% vxyz = r * [u0(4); 0; u0(5)];
vxyz = r * u0(4:6);
V = [u0(1:3); vxyz];

[P_pre, x_pre] = predict(2, V, Q, P, x, measurement_smooth, M, 0);

for j = 1:M
    feature_i(:, j) = x_pre(6 + 3 * j - 2 : 6 + 3 * j);
end

% if x_pre(1) > pi/6 || x_pre(1) < -pi/6 || x_pre(2) > pi/6 || x_pre(2) < -pi/6
%     return;
% else
    d = 0;
    if op_noise == 2
        sigma_r0 = 0.01;
        sigma_r1 = 0.09;
        dr_sigma = (sigma_r1 - sigma_r0) / (range(2) - range(1));

        sigma_b0 = 0.01;
        sigma_b1 = 0.09;
        db_sigma = (sigma_b1 - sigma_b0) / (range(2) - range(1));

    else
        sigma_r_ = 0.05;
        sigma_b_ = 0.05;
    end
    sigma_Feat  =[];
    for j = 1:M
        rot = R(x_pre(3),'y',0)*R(x_pre(2),'p',0)*R(x_pre(1),'r',0);
        delta = x_pre(6+3*j-2:6+3*j) - x_pre(4:6);
        y = rot' * delta;
        q = sqrt(y'*y);

        Rr = R(x_pre(3),'y',0)*R(x_pre(2),'p',0)*R(x_pre(1),'r',1);
        Rp = R(x_pre(3),'y',0)*R(x_pre(2),'p',1)*R(x_pre(1),'r',0);
        Ry = R(x_pre(3),'y',1)*R(x_pre(2),'p',0)*R(x_pre(1),'r',0);
        Hij = [Rr'*delta Rp'*delta Ry'*delta, -rot', rot'];
        H_feat{i, j} = Hij;
        if q <= range(2) && q >= range(1) && y(1) > 0
            vis = visibility(feature_i, x_pre(4:6), j);
            if vis == 1
%                 for k = 1:length(vis_feat)
%                     constraint_edge = [constraint_edge; j, vis_feat(k)];
%                 end
%                 vis_feat = [vis_feat; j];
                        
                if op_noise == 2
                    sigma_r{j, i} = sigma_r0 + dr_sigma * max(0, q - range(1));
                    sigma_b{j, i} = sigma_b0 + db_sigma * max(0, q - range(1));
                else
                    sigma_r{j, i} = sigma_r_;
                    sigma_b{j, i} = sigma_b_;
                end
                sigma_Feat = [sigma_Feat; sigma_r{j, i}^2; sigma_b{j, i}^2; sigma_r{j, i}^2];
                visble(j, i) = 1;
            else
                visble(j, i) = 0;
            end
        else
            visble(j, i) = 0;
        end
    end
%     if op == 2
%         N_cons = sparse(sigma_cons^2*eye(3*length(constraint_edge)));
        N_ = blkdiag(diag(sigma_Feat), N_cons);
%         N = diag(sigma_Feat);
%         for j = 1:size(constraint_edge, 1)
%             N = blkdiag(N, sigma_cons^2*eye(3));
%         end
        [K, H] = KF_gain(P_pre, N_, visble(:, i), constraint_edge, M, H_feat, i);
        PX=(eye(size(P_pre,1))-K*H)*P_pre;
%         obj = det(PX);
        obj = trace(PX);
%     end
% end
end

