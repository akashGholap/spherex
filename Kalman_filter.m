function [x_cm,y_cm,z_cm]=Kalman_filter(vi,tf,yi,x0)
Ac = [0 0 0;0 0 0;0 0 0];
Bc = [0;0;-1.622];
Cc = [1 0 0;0 1 0;0 0 1];
Dc = 0;
sysC = ss(Ac, Bc, Cc, Dc);
sysD = c2d(sysC,0.0001);
A = sysD.A;
B = sysD.B;
C = sysD.C;
D = sysD.D;
%%
dt = 0.0001;
Tfinal = tf;
t = 0:dt:Tfinal;
% u = ones(1,numel(t));
% x0 = vi;
% x = lsim(sysC,u,t,x0);
%%
u = 1;
Q = [1e-4 1e-4 1e-4;1e-4 1e-4 1e-4;1e-4 1e-4 1e-4];
R = [0.1 0 0;0 0.1 0;0 0 0.1];
x_(:,1) = [0;0;0];
xhat(:,1) = [0;0;0];
P = [0.1 0 0;0 0.1 0;0 0 0.1];
z(:,1) = [normrnd(vi(1),0.1);normrnd(vi(2),0.1);normrnd(vi(3),0.1)];
t(1) = 0;
v0 = xhat;
x_cm(1) = x0(1)+v0(1)*t(1);
y_cm(1) = x0(2)+v0(2)*t(1);
z_cm(1) = x0(3)+v0(3)*t(1)-0.5*1.622*t(1)^2;
%%
for i = 2:length(t)
    t(i) = t(i-1)+dt;
    x_(:,i) = A*xhat(:,i-1)+B*u;
    P = A*P*A'+Q;
    K = P*C'*inv(C*P*C'+R);
    z(:,i) = [normrnd(yi(i,4),0.05);normrnd(yi(i,5),0.05);normrnd(yi(i,6),0.05)];
    xhat(:,i)= x_(:,i)+K*(z(:,i)-C*x_(:,i));
    P = (eye(3,3)-K*C)*P;
    v0(1) = xhat(1,i);
    v0(2) = xhat(2,i);
    v0(3) = xhat(3,i)+1.622*t(i);
    x_cm(i) = x0(1)+v0(1)*t(i);
    y_cm(i) = x0(2)+v0(2)*t(i);
    z_cm(i) = x0(3)+v0(3)*t(i)-0.5*1.622*t(i)^2;
end
end