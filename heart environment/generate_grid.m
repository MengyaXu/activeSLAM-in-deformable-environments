clc;
clear;
close all;

% id = 1;
pos = [];
for x = -5:5:25
    for y = -15:5:15
        for z = -5:5:20
            if ismember(x, [0:15]) && ismember(y, [-6:6]) && ismember(z, [0:15])
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