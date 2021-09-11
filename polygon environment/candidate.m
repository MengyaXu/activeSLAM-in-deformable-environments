function [V_f, Xr_f] = candidate(V, Xr)
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
% face
% vertices'
% size(Xr(4:6, :)')
[d, ~] = point2trimesh('Face', face, 'Vertices', vertices', 'QueryPoints', Xr(4:6, :)');
[d_pre, ~] = point2trimesh('Face', face, 'Vertices', v_pre', 'QueryPoints', Xr(4:6, :)');
% d
% d_pre

index = find(abs(d) <= delta);
index_pre = find(abs(d_pre) <= delta);
% size(index)
% size(index_pre)
index = [index; index_pre];
index = unique(index);

Xr(:, index) = [];
V(:, index) = [];

V_f = V;
Xr_f = Xr;
end

