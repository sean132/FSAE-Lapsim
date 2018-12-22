function car = makeGG(paramArr,car)
ggpoints = [];
longAccelLookup = []; %maxLongAccel = f(latAccel,velocity)
longDecelLookup = []; %maxLongDecel = f(latAccel,velocity)
arr = reshape(paramArr,[numel(paramArr) 1]);
for i = 1:numel(arr)
    pSet = arr(i);
    p1 = []; p2 = []; p3 = []; p4 = [];
    %x: long %y: lat %z: velo
    if pSet.maxLatFlag == 1 && pSet.maxLongFlag == 1
        p1 = [pSet.maxLongLongAccel pSet.maxLongLatAccel pSet.longVel];
        p2 = [pSet.maxLongLongAccel -pSet.maxLongLatAccel pSet.longVel];
    end
    if pSet.maxLatFlag == 1 && pSet.maxBrakeFlag == 1
        p3 = [pSet.maxBrakeLongDecel pSet.maxBrakeLatAccel pSet.longVel];
        p4 = [pSet.maxBrakeLongDecel -pSet.maxBrakeLatAccel pSet.longVel];
    end
    ggpoints = [ggpoints; p1; p2; p3; p4];
    longAccelLookup = [longAccelLookup; p1];
    longDecelLookup = [longDecelLookup; p2];
end
car.ggPoints = ggpoints;
car.longAccelLookup = longAccelLookup;
car.longDecelLookup = longDecelLookup;
% gg = Polyhedron('V',ggpoints);
% gg.minVRep();
% figure(123);clf;
% scatter3(ggpoints(:,1),ggpoints(:,2),ggpoints(:,3),'+')
% % gg.plot()
% xlabel('Long');
% ylabel('Lat');
% zlabel('Velocity');