function [V_f, Xr_f] = candidate2(V, Xr, goal_pos)
delta = 0.3;
load data x measurement_smooth
load dataFeature id1 id2 tri1 tri2

face = [tri1; tri2];
for i = 1:length(tri1(:, 1))
    for j = 1:3
        face(i, j) = id1(tri1(i, j));
%         f(:, j) = x_pre(6 + 3*face(i, j)-2 : 6 + 3*face(i, j));
%         f_old(:, j) = x(6 + 3*face(i, j)-2 : 6 + 3*face(i, j));
    end
end
for i = 1:length(tri2(:, 1))
    for j = 1:3
        face(length(tri1(:, 1))+i, j) = id2(tri2(i, j));
    end
end

xf = x(7:end);
xf_pre = xf + measurement_smooth;
vertices = zeros(3, length(xf)/3);
v_pre = vertices;
for i = 1:length(xf)/3
    vertices(:, i) = xf(3 * i - 2 : 3 * i);
    v_pre(:, i) = xf_pre(3 * i - 2 : 3 * i);
end
d = [];
d_pre = [];
d_goal = [];
for i = 1:length(Xr(1, :))
    xr = Xr(4:6, i);
    [di, ~] = point2trimesh('Face', face, 'Vertices', vertices', 'QueryPoints', xr');
    d = [d; di];
    [di_pre, ~] = point2trimesh('Face', face, 'Vertices', v_pre', 'QueryPoints', xr');
    d_pre = [d_pre; di_pre];
    di_goal = norm(goal_pos - xr);
    d_goal = [d_goal; di_goal];
end


[d, ~] = point2trimesh('Face', face, 'Vertices', vertices', 'QueryPoints', Xr(4:6, :)');
[d_pre, ~] = point2trimesh('Face', face, 'Vertices', v_pre', 'QueryPoints', Xr(4:6, :)');

index = find(abs(d) <= delta);
index_pre = find(abs(d_pre) <= delta);
% size(index)
% size(index_pre)
index = [index; index_pre];
index = unique(index);

Xr(:, index) = [];
V(:, index) = [];
d_goal(index) = [];
[d_g, id_g] = sort(d_goal);
l = length(Xr(1, :));

V_f = V(:, id_g(1: l/10));
Xr_f = Xr(:, id_g(1: l/10));
end

