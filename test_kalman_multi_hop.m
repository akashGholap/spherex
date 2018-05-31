clear all
global Ri
global Rf
global g
r_i = [0 0 0];
r_f = [2 3 0];
x_i = r_i;
g = 1.622;
dyn = [];
est = [];
init = r_i;
fin = r_f;
for i = 1
    Ri = r_i(1:2);
    Rf = r_f(1:2);
    R = Rf-Ri;
    x0=1;
    options =optimoptions(@fminunc,'Algorithm','quasi-newton');
    [t,fval,exitflag,output]=fminunc(@objfun,x0,options);
    vl = t*g/2;
    vR = R./t;
    v_i = [vR vl];
    y_init = zeros(6,1);
    y_init(1:3) = r_i';
    y_init(4:6) = v_i';
    ts = 0.0001;
    tspan=0:ts:20;
    options = odeset('RelTol',1e-3,'AbsTol', 1e-5,'Events', @EventFunction);
    [ti,yi]=ode45(@(ti,yi)hop(ti,yi,ts),tspan,y_init,options);
    [x_cm,y_cm,z_cm]=Kalman_filter(v_i',t,yi,x_i');
    dyn = [dyn;yi(:,1:3)];
    est = [est;[x_cm' y_cm' z_cm']];
    r_i = dyn(end,:);
    r_f = [r_i(1)+randi([1 10],1) r_i(2)+randi([1 10],1) r_i(3)];
    init = [init;r_i];
    fin = [fin;r_f];
    x_i = est(end,:);
    clear ti
    clear yi
    clear x_cm
    clear y_cm
    clear z_cm;
end
%%
figure()
plot3(dyn(:,1),dyn(:,2),dyn(:,3),'b')
hold on
plot3(est(:,1),est(:,2),est(:,3),'r')
axis equal
grid minor
set(gca,'Fontsize',13)
xlabel('x (m)')
ylabel ('y (m)')
zlabel('z (m)')
legend('Real position','Kalman filter estimate')
%%
for i = 1:length(est)
    error(:,i) = [dyn(i,1)-est(i,1);dyn(i,2)-est(i,2);dyn(i,3)-est(i,3)];
    tot_err(i) = norm(error(:,i));
end
%%
t = 0:1/10^4:(length(est)-1)/10^4;
figure()
subplot(3,1,1)
plot(t,error(1,:),'k')
set(gca,'Fontsize',13)
xlabel('Time (s)')
ylabel('x-error (m)')
xlim([0 length(t)/10^4])
subplot(3,1,2)
plot(t,error(2,:),'k')
set(gca,'Fontsize',13)
xlabel('Time (s)')
ylabel('y-error (m)')
xlim([0 length(t)/10^4])
subplot(3,1,3)
plot(t,error(3,:),'k')
set(gca,'Fontsize',13)
% plot(ti(1:1868),tot_err,'r','Linewidth',2)
set(gca,'Fontsize',13)
% xlim([0 1.01])
% legend('x error','y error','z error')
xlabel('Time (s)')
ylabel('z-error (m)')
xlim([0 length(t)/10^4])