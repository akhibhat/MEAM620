function neighbours = getneighbours(map,index,allPoints)

% index = 1;

x_neigh = [1 0 0];
y_neigh = [0 1 0];
z_neigh = [0 0 1];


lowbound = map.bound_xyz(1:3);
upbound = map.bound_xyz(4:6);

point = allPoints(index).point;

vpoint = pos2sub(map,point);
lowsub = pos2sub(map,lowbound);
upsub = pos2sub(map,upbound);

nbours = [];
neighbour = [];
neighbour2 = [];
neighbours = [];

nbours(1,:) = vpoint + x_neigh;
nbours(2,:) = vpoint - x_neigh;
nbours(3,:) = vpoint + y_neigh;
nbours(4,:) = vpoint - y_neigh;
nbours(5,:) = vpoint + z_neigh;
nbours(6,:) = vpoint - z_neigh;

j = 1;
for rows = 1:6
    if ((sum(nbours(rows,:)>=lowsub) == 3) && (sum(nbours(rows,:)<=upsub) == 3))
       neighbour(j,:) = nbours(rows,:);
       j = j + 1;
    else
        continue;
    end
end

%%
j = 1;
for i = 1:size(neighbour,1)
    if map.occgrid(neighbour(i,1),neighbour(i,2),neighbour(i,3))==0
        neighbour2(j,:) = neighbour(i,:);
        j = j+1;
    else
        continue;
    end
end
%%

for i = 1:size(neighbour2,1)
    neighbours(i) = sub2ind(size(map.occgrid),neighbour2(i,1),neighbour2(i,2),neighbour2(i,3));
end