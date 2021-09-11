function R = Rotation(theta, op, j)

if j == 0
    if op == 'r'
        R = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
    elseif op == 'p'
        R = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    elseif op == 'y'
        R = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
    end
elseif j == 1
    if op == 'r'
        R = [1 0 0; 0 -sin(theta) -cos(theta); 0 cos(theta) -sin(theta)];
    elseif op == 'p'
        R = [-sin(theta) 0 cos(theta); 0 1 0; -cos(theta) 0 -sin(theta)];
    elseif op == 'y'
        R = [-sin(theta) -cos(theta) 0; cos(theta) -sin(theta) 0; 0 0 1];
    end
end
end

