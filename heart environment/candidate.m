function [V_f, Xr_f] = candidate(V, Xr)
delta = 3;
load data x measurement_smooth
load dataFeature tri1

face = tri1;

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

