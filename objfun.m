function f = objfun(x)
global Ri
global Rf
global g
R = Rf-Ri;
f = (R(1)/x(1))^2+(R(2)/x(1))^2+(x(1)*g/2)^2;
end