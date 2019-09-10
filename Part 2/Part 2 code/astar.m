% function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an mx3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path. The first
%   row is start and the last row is goal. If no path is found, PATH is a
%   0x3 matrix. Consecutive points in PATH should not be farther apart than
%   neighboring voxels in the map (e.g. if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of nodes that were expanded while performing the search.
%   
% paramaters:
%   map     - the map object to plan in
%   start   - 1x3 vector of the starting coordinates [x,y,z]
%   goal:   - 1x3 vector of the goal coordinates [x,y,z]
%   astar   - boolean use astar or dijkstra

% if nargin < 4
%     astar = false;
% end
tic
astarvar = 1;

% Map 0
start = [10 4 0];
goal = [1.25 2 2];

% Map 1
% start = [0 10 0];
% goal = [4 17 2];

% Map 3 (Empty map)
% start = [0 10 0];
% goal = [4 17 2];

map = load_map('map0.txt', 0.25,1,0);

startidx = pos2ind(map,start);
goalidx = pos2ind(map,goal);

boundary = map.bound_xyz;
grid = map.occgrid;
xyz_res = map.res_xyz;

maxy = size(grid,1);
maxx = size(grid,2);
maxz = size(grid,3);
totalidx = maxy*maxx*maxz;

obstaclearray = find(map.occgrid == 1);

eligiblepoints = totalidx - size(obstaclearray,1);
nextindexcost = ones(eligiblepoints,2);

allPoints = struct;
j = 1;
for i = 1:totalidx
    allPoints(i).index = i;
    allPoints(i).totalcost = inf;
    allPoints(i).point = ind2pos(map,i);
    if isempty(find(obstaclearray == i, 1))
        allPoints(i).visnovisobs = -1;
        nextindexcost(j,1) = inf;
        nextindexcost(j,2) = i;
        j = j+1;
    else
        allPoints(i).visnovisobs = 0;
    end
end


%%
if (allPoints(startidx).visnovisobs == 0) || (allPoints(goalidx).visnovisobs == 0)
    disp('Start point or goal point in obstacle');
else
    point = start;
    startidx = pos2ind(map,point);
    allPoints(startidx).totalcost = 0 + heuristic(astarvar,goal,point);
    allPoints(startidx).parent = -1;
    nbouridx = getneighbours(map,startidx,allPoints);
    allPoints(startidx).nbouridx = nbouridx;
    allPoints(startidx).visnovisobs = 1;
    delidx = find(nextindexcost(:,2) == startidx);
    nextindexcost(delidx,:) = [];
    
    for i = 1:size(nbouridx,1)
        allPoints(nbouridx(i)).parent = startidx;
        allPoints(nbouridx(i)).totalcost = allPoints(startidx).totalcost + 1 + heuristic(astarvar,goal,allPoints(nbouridx(i)).point);
        updateidx = find(nextindexcost(:,2) == nbouridx(i));
        nextindexcost(updateidx,1) = allPoints(nbouridx(i)).totalcost;
    end
    
    [mincost, costindex] = min(nextindexcost(:,1));
    nextindex = nextindexcost(costindex,2);
    num_expanded = 1;
    pathnotfound = 0;
    
    
 
    while (nextindex ~= goalidx) && (pathnotfound == 0)
        allPoints(nextindex).visnovisobs = 1;
        nbours = getneighbours(map,nextindex,allPoints);
        nextnbouridx = pos2ind(map,nbours);
        allPoints(nextindex).nbouridx = nextnbouridx;
        
        
        for i = 1:size(nextnbouridx,1)
            if allPoints(nextnbouridx(i)).visnovisobs == -1
                allPoints(nextnbouridx(i)).parent = nextindex;
                allPoints(nextnbouridx(i)).totalcost = allPoints(nextindex).totalcost + 1 + heuristic(astarvar,goal,allPoints(nextnbouridx(i)).point);
                updateidx = find(nextindexcost(:,2) == nextnbouridx(i));
                nextindexcost(updateidx,1) = allPoints(nextnbouridx(i)).totalcost;
            else
                c2r = allPoints(nextnbouridx(i)).totalcost;
                newc2r = allPoints(nextindex).totalcost + 1 + heuristic(astarvar,goal,allPoints(nextnbouridx(i)).point);
                if newc2r < c2r
                    c2r = newc2r;
                    allPoints(nextnbouridx(i)).parent = nextindex;
                    allPoints(nextnbouridx(i)).totalcost = newc2r;
                else
                    continue;
                end
            end
        end
        
        delidx = find(nextindexcost(:,2) == nextindex);
        nextindexcost(delidx,:) = [];
        
        [mincost, costindex] = min(nextindexcost(:,1));
        if mincost == inf
            pathnotfound = 1;
            break;
        end
        nextindex = nextindexcost(costindex,2);
        num_expanded = num_expanded + 1;
    end
    if pathnotfound == 0
        allPoints(nextindex).visnovisobs = 1;
        path = getpath(allPoints,nextindex); 
    else
        path = int16.empty(0,3);
    end
%     allPoints(nextindex).visnovisobs = 1;
%     path = getpath(allPoints,nextindex); 
end
toc