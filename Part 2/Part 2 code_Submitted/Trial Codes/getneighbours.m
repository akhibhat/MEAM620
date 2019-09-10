function neighbours = getneighbours(map,index)

% index = 1;

ymax = size(map.occgrid,2);
xmax = size(map.occgrid,1);
zmax = size(map.occgrid,3);

[y, x, z] = ind2sub([ymax,xmax,zmax], index);

x_neigh = [1 0 0];
y_neigh = [0 1 0];
z_neigh = [0 0 1];
% 
% 
% lowbound = map.bound_xyz(1:3);
% upbound = map.bound_xyz(4:6);
% 
% point = allPoints(index).point;
% 
% vpoint = pos2sub(map,point);
% lowsub = pos2sub(map,lowbound);
% upsub = pos2sub(map,upbound);

nbours = [];
neighbour = [];
neighbours = [];

nbours(1,:) = [y x z] + x_neigh;
nbours(2,:) = [y x z] - x_neigh;
nbours(3,:) = [y x z] + y_neigh;
nbours(4,:) = [y x z] - y_neigh;
nbours(5,:) = [y x z] + z_neigh;
nbours(6,:) = [y x z] - z_neigh;

j = 1;
for rows = 1:6
    if ((sum(nbours(rows,:)>=[1 1 1]) == 3) && (sum(nbours(rows,:)<=[ymax xmax zmax]) == 3))
       neighbour(j,:) = nbours(rows,:);
       j = j + 1;
    else
        continue;
    end
end

%%
j = 1;
for i = 1:size(neighbour,1)
    ind = sub2ind([xmax ymax zmax],neighbour(i,2), neighbour(i,1), neighbour(i,3));
    if map.occgrid(ind)==0
        neighbours(j)=ind;
        j=j+1;
    end
end
% for i = 1:size(neighbour,1)
%     if map.occgrid(neighbour(i,1),neighbour(i,2),neighbour(i,3))==0
%         neighbour2(j,:) = neighbour(i,:);
%         j = j+1;
%     else
%         continue;
%     end
% end
% %%
% 
% for i = 1:size(neighbour2,1)
%     neighbours(i) = sub2ind(size(map.occgrid),neighbour2(i,2),neighbour2(i,1),neighbour2(i,3));
% end