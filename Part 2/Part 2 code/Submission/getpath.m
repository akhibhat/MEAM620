function path = getpath(allPoints,finalidx)

reversepath(1) = finalidx;
i =2;

index = finalidx;
while allPoints(index).parent ~= -1
    reversepath(i) = allPoints(index).parent;
    index = reversepath(i);
    i = i+1;
end

pathindex = fliplr(reversepath);

for i = 1:size(pathindex,2)
    path(i,:) = allPoints(pathindex(i)).point;
end
end