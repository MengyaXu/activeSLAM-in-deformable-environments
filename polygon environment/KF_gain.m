function [K,H]=KF_gain(P_pre, N, visble, constraint_edge, Num_feature, H_feat, i)
rows=[];
columns=[];
values=[];
k=0;

for j=1:1:Num_feature
    if visble(j)==1
        k=k+1;
        rows=[rows,(3*k-2)*ones(1, 9),(3*k-1)*ones(1, 9), 3*k*ones(1,9)];
        columns=[columns,1:6,6+3*j-2:6+3*j,1:6,6+3*j-2:6+3*j,1:6,6+3*j-2:6+3*j];
        values=[values, H_feat{i, j}(1,:), H_feat{i, j}(2,:), H_feat{i, j}(3,:)];
    end
end
for j=1:1:size(constraint_edge,1)
    in=constraint_edge(j,1);
    out=constraint_edge(j,2);
    rows=[rows,(3*k+3*j-2)*ones(1,2),(3*k+3*j-1)*ones(1,2),(3*k+3*j)*ones(1,2)];
    columns=[columns,6+3*in-2,6+3*out-2,6+3*in-1,6+3*out-1,6+3*in,6+3*out];
    values=[values,-1, 1, -1, 1, -1, 1];
end
H=sparse(rows,columns,values,3*k+size(constraint_edge,1)*3,size(P_pre,1));
K=P_pre*H'/(H*P_pre*H'+N);
end

