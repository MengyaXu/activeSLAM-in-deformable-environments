function [P_pre,x_pre]=predict(op, u, Q, P, x, measurement_smooth, M, noise_odem)
r=R(x(3),'y',0)*R(x(2),'p',0)*R(x(1),'r',0);
motion = [u(1:3);r * u(4:6); measurement_smooth];
if op == 1  % actual pose
    x_pre = x + motion(1:6) + noise_odem;
    P_pre = [];
elseif op == 2  % predict vector
    x_pre = x + motion;
    
    Rr = R(x(3),'y',0)*R(x(2),'p',0)*R(x(1),'r',1);
    Rp = R(x(3),'y',0)*R(x(2),'p',1)*R(x(1),'r',0);
    Ry = R(x(3),'y',1)*R(x(2),'p',0)*R(x(1),'r',0);
    Fx = [eye(3,3) zeros(3,3);
        Rr*u(4:6) Rp*u(4:6) Ry*u(4:6)  eye(3)];
    F = sparse(blkdiag(Fx, eye(3*M)));
    G = sparse(blkdiag(eye(3,3), r, eye(3*M)));
    P_pre=F*P*F' + G*Q*G';
end
x_pre(1) = wrapToPi(x_pre(1));
x_pre(2) = wrapToPi(x_pre(2));
x_pre(3) = wrapToPi(x_pre(3));