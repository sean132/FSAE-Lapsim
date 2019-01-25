clear; clc
n = 10;
pos = sdpvar(1,n);
u = sdpvar(1,n);
constr = [];
holder = 0;
for i = 2:n
    a = advState(pos(i),pos(i-1),u(i));
    constr = [constr; a;u(i) <= holder; u(i) >= -1];
    holder = holder+1;
end
constr = [constr; pos(1) == 0;u(1) <= 1; u(1) >= -1];
goalXPos = 10^3;
obj = goalXPos - pos(n);
options = sdpsettings('solver','fmincon','verbose',1);
diag = optimize(constr, obj, options);
