function finalpath = prune_neigh(map,mapblocks,xyres,path1)

wpts = size(path1,1);

fin_path(1,:) = path1(1,:);

i = 2;
j = 3;
x = 2;
while j ~= wpts && i ~=wpts
    diff = (path1(j,:) - path1(i,:))/(j-i);
    predir = vecnorm(diff,2,1);
    j = j + 1;
    change = 0;
    while change == 0 && j<wpts
        newdiff = (path1(j,:) - path1(i,:))/(j-i);
        dir = vecnorm(newdiff,2,1);
        if sum(round(dir,4) == round(predir,4)) == 3
            change = 0;
            j = j + 1;
        else
            change = 1;
            fin_path(x,:) = path1(i,:);
            fin_path(x+1,:) = path1(j-1,:);
            i = j-1;
            j = i + 1;
            x = x +1;
        end
    end 
end
fin_path(end+1,:) = path1(end,:);

blocks = mapblocks;

for i = 1:size(blocks,1)
    block_centres(i,:) = [(blocks(i,4)-blocks(i,1))/2,(blocks(i,5)-blocks(i,2))/2,(blocks(i,6)-blocks(i,3))/2];
    block_diag(i) = (norm(blocks(i,4:6)-blocks(i,1:3))/2) + xyres ;
end

newwpts = size(fin_path,1);
j = 1;
k = 3;
x = 1;
while j <=newwpts && k<=newwpts
    v1 = fin_path(j,:);
    v2 = fin_path(k,:);
    collided = findcollision(map,v1,v2);
    if collided == 0
        k = k+1;
    else
        finalpath(x,:) = fin_path(j,:);
        finalpath(x+1,:) = fin_path(k-1,:);
        j = k-1;
        k = j+2;
        x = x+1;
    end
end

finalpath(end+1,:) = fin_path(end,:);

end