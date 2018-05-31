function dy = hop(t,y,ts)
global g
dy=zeros(6,1);
r = y(1:3);
v = y(4:6);
dy(1:3) = v;
dy(4:6) = [0;0;-g];
end