clc;
clear;
close all;

% id = 1;
pos = [];
for x = -1.5:4.5
    for y = -3:3
        for z = -2:6
            if ismember(x, [0.5:2.5]) && ismember(y, [-1:1]) && ismember(z, [0:4])
%                 pos = [pos [0;0;0]];
            else
                pos = [pos [x; y; z]];
%                 pos{id} = [x;y;z];
%                 id = id + 1;
            end
        end
    end
end
pos
save grid pos