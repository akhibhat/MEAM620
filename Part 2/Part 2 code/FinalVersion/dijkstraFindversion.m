function [path, num_expanded] = dijkstraFindversion(map, start, goal, astarvar)
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

if nargin < 4
    astarvar = false;
end

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

allPoints = struct;

for i = 1:totalidx
    allPoints(i).index = i;
    allPoints(i).totalcost = inf;
    allPoints(i).point = ind2pos(map,i);
    if isempty(find(obstaclearray == i, 1))
        allPoints(i).visnovisobs = -1;
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
    nbouridx = get_neighbours(map,startidx);
    allPoints(startidx).visnovisobs = 1;
    mincostarray = [];
    for i = 1:size(nbouridx,2)
        allPoints(nbouridx(i)).parent = startidx;
        allPoints(nbouridx(i)).totalcost = allPoints(startidx).totalcost + 1 + heuristic(astarvar,goal,allPoints(nbouridx(i)).point);
        mincostarray = vertcat(mincostarray,[allPoints(nbouridx(i)).totalcost, nbouridx(i)]);    
    end
    
    mincost = find(mincostarray(:,1)==min(mincostarray),1);
    
    nextindex = mincostarray(mincost,2);
    mincostarray(mincost,:) = [];
    num_expanded = 1;
    pathnotfound = 0;
    
    while (nextindex ~= goalidx) && (pathnotfound == 0)
        nextnbouridx = get_neighbours(map,nextindex);
        allPoints(nextindex).visnovisobs = 1;
        
        for i = 1:size(nextnbouridx,2)
            tcost = allPoints(nextindex).totalcost + 1 + heuristic(astarvar,goal,allPoints(nextnbouridx(i)).point);
            precost = allPoints(nextnbouridx(i)).totalcost;
            if precost > tcost
                allPoints(nextnbouridx(i)).totalcost = tcost;
                allPoints(nextnbouridx(i)).parent = nextindex;
                
                mincostarray = vertcat(mincostarray,[tcost nextnbouridx(i)]);
            end
        end

        if ~isempty(mincostarray)
            mincost = find(mincostarray(:,1)==min(mincostarray),1);
    
            nextindex = mincostarray(mincost,2);
            mincostarray(mincost,:) = [];
        else
            pathnotfound = 1;
        end
        num_expanded = num_expanded + 1;
    end
    if isempty(mincostarray)
        path = int16.empty(0,3);
    else
        path = getpath(allPoints,nextindex,goal,start);
    end
end