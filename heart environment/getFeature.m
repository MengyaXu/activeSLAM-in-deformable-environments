clc;
clear;
close all;
[x, y, z, c] = stlread('Heart.stl');
tran = [5; 18; 110]';
f_all_i = x.Points/10 + tran;
tri = x.ConnectivityList;
trisurf(tri, f_all_i(:, 1), f_all_i(:, 2), f_all_i(:, 3));
% f_all_i(2:2:end, :) = [];
M_all = length(f_all_i(:, 1));
n = 0;
for j = 1:M_all
    feature_all{1}(1:3, j) = f_all_i(j, :)';
    if mod(j, floor(M_all/60)) == 3
        n = n + 1;
        feature_i(n, :) = f_all_i(j, :);
    end
end
del = DelaunayTri(feature_i(:, 1), feature_i(:, 2), feature_i(:, 3));
tri1 = del.freeBoundary();
a = unique(tri1);
M = length(a);
c= [];
for j = 1:length(feature_i(:, 1))
    b = find(a == j);
    if length(b)  == 0
        c = [c j];
    end
end
feature_i(c, :) = [];
for j = 1:M
    feature{1}(:, j) = feature_i(j, :)';
end

id1 = 1:M;
del = DelaunayTri(feature_i(:, 1), feature_i(:, 2), feature_i(:, 3));
tri1 = del.freeBoundary();

figure(2)
trisurf(tri1, feature_i(:, 1), feature_i(:, 2), feature_i(:, 3));

id2 = [];
tri2 = [];

save dataFeature M feature M_all feature_all id1 id2 tri1 tri2 -append
% 
% axis([-4 4 -4 4 -4 4]);