function v = getV_trace(op, x)
load data op_trans i
dr = 2.5;
lr = 0.05;
dtheta = pi/10;
if op == 1
    if op_trans == 1
        if i == 2
            v = [0; 0; 0; dr+pi/2; 0; lr];
        else
            v = [0; 0; 0; dr; 0; lr];
        end
    elseif op_trans == 2
%         v = [0; 0; 0; 0; 0; 0.5];
        v = [0; 0; pi/12; sqrt(3)/4; -dr; lr];
%         r = R(dtheta,'y',0)*R(0,'p',0)*R(0,'r',0);
%         xyz = r*[dr; 0; lr];
%         v = [0; 0; dtheta; xyz];
        
%         rr = 5;
%         v = [dtheta; rr*(sin(i*dtheta)-sin((i-1)*dtheta)); rr*(cos((i-1)*dtheta)-cos(i*dtheta))];
    end
elseif op == 2
%     u0 = zeros(6, 1);
%     [u, fmin] = fmincon('objFun2', u0, [],[],[],[],[-pi/6,-pi/6,-pi/4,-1,-0.5,-0.5],[pi/6,pi/6,pi/4,1,0.5,0.5],'nlc');
%     if u(1) + x(1) > pi/6 || u(1) + x(1) < -pi/6 || u(2) + x(2) > pi/6 || u(2) + x(2) < -pi/6
%         b = 1
%     end
%     r=R(u(3),'y',0)*R(u(2),'p',0)*R(u(1),'r',0);
% %     vxyz = r * [u(4); 0; u(5)];
%     vxyz = r * u(4:6);
%     v = [u(1:3); vxyz];
    
    min = Inf;
    rr = -pi/6:pi/6:pi/6;
    rp = -pi/6:pi/6:pi/6;
    ry = -pi/4:pi/8:pi/4;
    dx = -3:1.5:3;
%     dy = -2*l:1*l/2:2*l;
    dz = -2:2:2;
    
    count = 0;
    Xr = [];
    V = [];
    for roll = 1:length(rr)
        if ry(roll) + x(1) > pi/6 || ry(roll) + x(1) < -pi/6
        else
            for pitch = 1:length(rp)
                if rp(pitch) + x(2) > pi/6 || rp(pitch) + x(2) < -pi/6
                else
                    for yell = 1:length(ry)
                        for i = 1:length(dx)
    %                         for j = 1:length(dy)
                                for k = 1:length(dz)
                                    count = count + 1;
                                    r=R(ry(yell),'y',0)*R(rp(pitch),'p',0)*R(rr(roll),'r',0);
                                    vxyz = r * [dx(i); 0; dz(k)];
                                    v0 = [rr(roll); rp(pitch); ry(yell); vxyz];
                                    V = [V v0];
                                    
                                    rx = R(x(3),'y',0)*R(x(2),'p',0)*R(x(1),'r',0);
                                    motion = [rr(roll); rp(pitch); ry(yell); rx * vxyz];
                                    xr_pre = x(1:6) + motion;
                                    Xr = [Xr xr_pre];
                                    
%                                     [g, ~] = nlc(v0, count);
%                                     if g < 0
%                                         obj = objFun2(v0);
%                                         if obj <= min
%                                             min = obj;
%                                             v = v0;
%                                         end
%                                     end
                                end
    %                         end
                        end
                    end
                end
            end
        end
    end
    
%     tic;
    [V_f, Xr_f] = candidate(V, Xr);
%     toc;
%     t1 = toc
%     length(V_f(1, :))
    
%     tic;

% V_f = V;
    for i = 1:length(V_f(1, :))
%         tic;
        obj = objFun2(V_f(:, i));
%         toc;
%         t3 = toc
        if obj <= min
            min = obj;
            v = V_f(:, i);
        end
    end
%     toc;
%     t2 = toc
end

