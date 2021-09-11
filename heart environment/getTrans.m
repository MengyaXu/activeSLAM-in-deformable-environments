load dataFeature M_all feature_all
N = 6;
d = 0.5;
l = 0;
e = 0.5;
for i = 1:N/2
%     translation(1:3, i) = [d; 0; l];
    translation(1:3, i) = [0; 0; 0];
end
for i = N/2+1:N
%     translation(1:3, i) = -[d; 0; l];
    translation(1:3, i) = -[0; 0; 0];
end
for j = 1:M_all
    if j == 1
        for i = 1:N
            deform{j}(1:3, i) = [0; 0; 0];
        end
    else
        x = feature_all{1}(1, j) - feature_all{1}(1, 1);
        y = feature_all{1}(2, j) - feature_all{1}(2, 1);
%         x = feature_all{1}(1, j) - 5;
%         y = feature_all{1}(2, j) - 5;
        d_local = sqrt(x^2 + y^2);
        for i = 1:N/2
            if d_local == 0
                deform{j}(1:3, i) = i*[0; 0; e];
            else
                deform{j}(1:3, i) = i*[e/d_local*x; e/d_local*y; 0];
            end
        end
        for i = N/2+1:N
            if d_local == 0
                deform{j}(1:3, i) = (N-i+1)*[0; 0; e];
            else
                deform{j}(1:3, i) = (N-i+1)*[e/d_local*x; e/d_local*y; 0];
            end
        end
    end
end

save dataFeature translation deform -append