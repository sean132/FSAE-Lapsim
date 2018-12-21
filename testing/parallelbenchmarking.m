numIter = 3;
tic
for i = 1:numIter
   gg2(car); %out of date, gg2 is parallel now
end
t1 = toc;
tic
for i = 1:numIter
    gg2parall(car);
end
t2 = toc;
tic
for i = 1:numIter
    gg2parall2(car)
end
t3 = toc
fprintf("seq: %0.2f parallel: %0.2f parall2: %0.2f\n",[t1 t2 t3]);

%%
numIter = 1;
profile on

tic
for i = 1:numIter
    gg2parall(car);
end
t2 = toc;
profile viewer
%%
tic
gg2parall2(car);
toc