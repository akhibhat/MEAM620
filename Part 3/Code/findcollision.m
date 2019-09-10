function collides = findcollision(map,v1,v2)

blocks = map.blocks(:,1:6);

for i = 1:size(blocks,1)
    block_centres(i,:) = [(blocks(i,4)-blocks(i,1))/2,(blocks(i,5)-blocks(i,2))/2,(blocks(i,6)-blocks(i,3))/2];
end

for i = 1:size(blocks,1)
    dist(i,:) = point_to_line(block_centres(i,:),v1,v2);
end

[number, ~] = find(dist == min(dist));
block = blocks(number,:);

collides = 0;

for t = 1:size(block)

    minblock = block(1,1:3);
    maxblock = block(1,4:6);

    halfsides = ((maxblock-minblock)/2) + map.res_xyz(1);

    if sum(dist(number)<halfsides) == 3
        collides = 1;
    else

        block_points(1,:) = minblock;
        block_points(2,:) = maxblock;
        block_points(3,:) = [minblock(1), maxblock(2), minblock(3)];
        block_points(4,:) = [maxblock(1), minblock(2), maxblock(3)];
        block_points(5,:) = [minblock(1), minblock(2), maxblock(3)];
        block_points(6,:) = [maxblock(1), maxblock(2), minblock(3)];
        block_points(7,:) = [maxblock(1), minblock(2), minblock(3)];
        block_points(8,:) = [minblock(1), maxblock(2), maxblock(3)];

        for i = 1:8
            block_dist(i,:) = point_to_line(block_points(i,:),v1,v2);
        end

        [point, inside_mindist] = find(block_dist == min(block_dist));

        if rem(point,2) == 0
            p = point-1;
            op = point;
        else
            p = point;
            op = point+1;
        end

        lines = v2-v1;
        v1p = block_points(p,:)-v1;
        v1op = block_points(op,:)-v1;

        if (dot(cross(v1p,lines),cross(v1op,lines))) <0
            collides = collides +1;
        else
            collides = 0;
        end
    end
end
end