clear
clc
arr = 1:10;
K = 6;
combos = combnk(arr,K);
a = zeros(numel(arr).^K,1);
tic
parfor i = 1:10
    a(i)
end
toc
disp('done')
%%
clear
load surfdata.mat
A = Polyhedron('V',crust_matrix)
A.minVRep();
figure(123);clf;
A.plot()
