function v = getV_global(op, x, pos, goals, goal_index)

min = Inf;
rr = -pi/6:pi/6:pi/6;
rp = -pi/6:pi/6:pi/6;
ry = -pi/4:pi/8:pi/4;
dx = -0.2:0.1:0.2;
%     dy = -2*l:1*l/2:2*l;
dz = -0.5:0.5:0.5;

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


if op == 11
    [V_f, Xr_f] = candidate(V, Xr);
    goal = goals(goal_index);
    goal_pos = pos(:, goal);
    
    for i = 1:length(Xr_f(1, :))
        xr = Xr_f(4:6, i);
        d = norm(goal_pos - xr);
        if d <= min
            min = d;
            v = V_f(:, i);
        end
    end
elseif op == 21
    goal = goals(goal_index);
    goal_pos = pos(:, goal);
    [V_f, Xr_f] = candidate2(V, Xr, goal_pos);
    for i = 1:length(V_f(1, :))
    %         plot3([x(4) Xr_f(4, i)], [x(5) Xr_f(5, i)], [x(6) Xr_f(6, i)], 'k-');
        obj = objFun2(V_f(:, i));
        if obj <= min
            min = obj;
            v = V_f(:, i);
        end
    end
elseif op == 31
    [V_f, Xr_f] = candidate(V, Xr);
    goal = goals(goal_index);
    goal_pos = pos(:, goal);
    for i = 1:length(V_f(1, :))
        obj = objFun2(V_f(:, i));
        xr = Xr_f(4:6, i);
        d = norm(goal_pos - xr);  
        obj = obj + d;
        if obj <= min
            min = obj;
            v = V_f(:, i);
        end
    end
end
end

