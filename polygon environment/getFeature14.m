clc;
clear;
close all;

M = 14;
edge = [];
tran = [1.5; 0; 0];
feature_i = [0 0 0; 0 -1 1; 0.5 -0.5 1; 0.5 0.5 1; 0 1 1; -0.5 0.5 1; -0.5 -0.5 1; ...
    0 -1 3; 0.5 -0.5 3; 0.5 0.5 3; 0 1 3; -0.5 0.5 3; -0.5 -0.5 3; 0 0 4]' + tran;
% feature_i = [0 0 0; 0.5 -0.5 1; 0.5 0.5 1; -0.5 0.5 1; -0.5 -0.5 1; ...
%     0.5 -0.5 3; 0.5 0.5 3; -0.5 0.5 3; -0.5 -0.5 3; 0 0 4]';
for j = 1:M
    feature{1}(1:3, j) = feature_i(:, j);
    edge = [edge, feature{1}(1:3, j)];
end
% hedg = plot3(edge(1, :), edge(2, :), edge(3, :), '.--k', 'linewidth',1, 'MarkerSize', 15);
id1 = [1:5, 8:11, 14];
id2 = [1:2, 5:8, 11:14];
x1 = [feature_i(1, 1:5) feature_i(1, 8:11) feature_i(1, end)];
y1 = [feature_i(2, 1:5) feature_i(2, 8:11) feature_i(2, end)];
z1 = [feature_i(3, 1:5) feature_i(3, 8:11) feature_i(3, end)];
tri1 = delaunay(y1, z1);    % n * 3
trisurf(tri1,x1,y1,z1);
hold on;
x2 = [feature_i(1, 1:2) feature_i(1, 5:8) feature_i(1, 11:end)];
y2 = [feature_i(2, 1:2) feature_i(2, 5:8) feature_i(2, 11:end)];
z2 = [feature_i(3, 1:2) feature_i(3, 5:8) feature_i(3, 11:end)];
tri2 = delaunay(y2, z2);
trisurf(tri2,x2,y2,z2);

save dataFeature M feature id1 id2 tri1 tri2 -append
% 
% axis([-4 4 -4 4 -4 4]);