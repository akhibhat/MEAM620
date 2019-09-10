function [findneigh] = get_neighbours(map,i)

jmax=size(map.occgrid,2);
kmax=size(map.occgrid,1);
lmax=size(map.occgrid,3);
[k ,j, l]=ind2sub([kmax,jmax,lmax],i);
neigh = [[j k l]+[1 0 0];
            [j k l] + [-1 0 0];
            [j k l] + [0 1 0];
            [j k l] + [0 -1 0];
            [j k l] + [0 0 1];
            [j k l] + [0 0 -1];];

i=1;
neighbour=[];
for iter = 1:length(neigh)
    if sum((neigh(iter,:)<=[jmax kmax lmax]))==3 && sum((neigh(iter,:)>=[1 1 1]))==3
        neighbour(i,:) = neigh(iter,:);
        i=i+1;
    else 
        
    end
end
i=1;

findneigh=[];
for iter = 1:size(neighbour,1)
    j=sub2ind([kmax,jmax,lmax],neighbour(iter,2), neighbour(iter,1), neighbour(iter,3));
    if map.occgrid(j)==0
        findneigh(i)=j;
        i=i+1;
    end
end
end

        



