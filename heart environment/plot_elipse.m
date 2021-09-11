function pcov_1 = plot_elipse(X,PX)
N=200;
inc=2*pi/N;
K=1;
phi=0:inc:2*pi;
r=sqrtm(PX);%
a=K*r*[cos(phi);sin(phi)];
pcov_1(1,:)=2*a(1,:)+X(1);
pcov_1(2,:)=2*a(2,:)+X(2);
% f_cov_1=plot(0,0,'k');
% set(f_cov_1,'XData',pcov_1(1,:),'YData',pcov_1(2,:));
