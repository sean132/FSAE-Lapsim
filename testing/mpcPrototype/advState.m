function constr = advState(pos,posi,u)
%returns yalmip constraint matrix entries
force = struct();
force.F = 10;
m = 10;
dt = .01;
aArr = sdpvar(1,10);
aArr(1) = posi;
for i = 2:numel(aArr)
    aArr(i) = aArr(i-1)+1;
end
a = aArr(end);
constr = [pos == posi + (u*10/m)*dt^2];