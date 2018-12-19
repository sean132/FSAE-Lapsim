function points = makeGG(paramArr)
points = [];
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
    points = [points; p1; p2; p3; p4];
end

gg = Polyhedron('V',points);
gg.minVRep();
figure(123);clf;
% scatter3(points(:,1),points(:,2),points(:,3),'+')
gg.plot()
xlabel('Long');
ylabel('Lat');
zlabel('Velocity');