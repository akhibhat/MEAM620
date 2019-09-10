function path = getpath(allPoints,finalidx,goal,start)

reversepath(1) = finalidx;
i =2;

index = finalidx;
while allPoints(index).parent ~= -1
    reversepath(i) = allPoints(index).parent;
    index = reversepath(i);
    i = i+1;
end

pathindex = fliplr(reversepath);

path(1,:) = start;
for i = 1:size(pathindex,2)
    path(i+1,:) = allPoints(pathindex(i)).point;
end
path(size(pathindex,2)+2,:) = goal;
end