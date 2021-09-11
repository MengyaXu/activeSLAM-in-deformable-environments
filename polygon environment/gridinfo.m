function goals = gridinfo(pos, feature_i, vis_feat, grid_init)
load data range
grid_vis_num = zeros(length(pos(1, :)), 1);
grid_vis = zeros(length(pos(1, :)), length(feature_i(1,:)));
for i = 1:length(pos(1, :))
%     grid_vis{i} = [];
    for j = 1:length(feature_i(1,:))
        if ismember(j, vis_feat)
        else
            grid_pos = pos(:, i);
            vis = visibility(feature_i, grid_pos, j);
            d = norm(grid_pos - feature_i(:, j));
            if vis == 1 && d >= range(1) && d <= range(2)
%                 grid_vis{i} = [grid_vis{i}; j];
                grid_vis_num(i) = grid_vis_num(i) + 1;
                grid_vis(i, j) = 1;
            end
        end
    end
end
vis_sum = sum(grid_vis_num);
% sample -> vis_number -> pos_id
samples = randsrc(1, 50, [1:length(pos(1, :)); grid_vis_num'/sum(grid_vis_num)]);
goal_num = inf;
goals = [];
for i = 1:length(samples)
    grid_id = samples(i);
    goal_sample{i} = [grid_init; grid_id];
    
    d = norm(pos(:, grid_id) - pos(:, grid_init));
    vis = inter(feature_i, pos(:, grid_id), pos(:, grid_init));
    if vis == 0
        c = 1;
    else
        c = 0;
    end
    
    grid_vis_pre = grid_vis;
    index = grid_vis_pre(grid_id, :) == 1;
    grid_vis_pre(:, index) = 0;
    grid_vis_pre(grid_id, :) = 0;
    grid_vis_num = sum(grid_vis_pre, 2);
    while sum(grid_vis_num) ~= 0
        next_id = randsrc(1, 1, [1:length(pos); grid_vis_num'/sum(grid_vis_num)]);
        goal_sample{i} = [goal_sample{i}; next_id];
        
        d = d + norm(pos(:, goal_sample{i}(end - 1)) - pos(:, next_id));
        vis = inter(feature_i, pos(:, goal_sample{i}(end - 1)), pos(:, next_id));
        if vis == 0
            c = c + 1;
        end
        
        index = grid_vis_pre(next_id, :) == 1;
        grid_vis_pre(:, index) = 0;
        grid_vis_pre(grid_id, :) = 0;
        grid_vis_num = sum(grid_vis_pre, 2);
    end
    
%     goal_num_ = length(goal_sample{i});
    goal_num_ = d + c;
    if goal_num_ < goal_num
        goal_num = goal_num_;
        goals = goal_sample{i};
    end
end
% order -> pos_id
end

