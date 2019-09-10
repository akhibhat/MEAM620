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
astar = 0;

% Map 0
% start = [10 4 0];
% goal = [1.25 2 2];

% Map 1
start = [5 -1 1];
goal = [4 17 2];

map = load_map('map1.txt', 0.25,1,0.1);

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
    allPoints(i).cost2reach = inf;
    allPoints(i).point = ind2pos(map,i);
    if isempty(find(obstaclearray == i, 1))
        allPoints(i).visnovisobs = -1;
    else
        allPoints(i).visnovisobs = 0;
    end
end

if (allPoints(startidx).visnovisobs == 0) || (allPoints(goalidx).visnovisobs == 0)
    disp('Start point or goal point in obstacle');
else
    point = start;
    startidx = pos2ind(map,point);

    allPoints(startidx).cost2reach = 0;
    % allPoints(startidx).c2goal = heuristic(astar);
    allPoints(startidx).parent = -1;
    neighbours = getneighbours(map,startidx,allPoints);
    nbouridx = pos2ind(map,neighbours);
    allPoints(startidx).nbouridx = nbouridx;
    allPoints(startidx).visnovisobs = 1;

    for i = 1:size(nbouridx,1)
        allPoints(nbouridx(i)).parent = startidx;
        allPoints(nbouridx(i)).cost2reach = allPoints(startidx).cost2reach + 1;
    end
    
    nextindex = getnextidx(allPoints);
    
    while nextindex ~= goalidx
        allPoints(nextindex).visnovisobs = 1;
        nbours = getneighbours(map,nextindex,allPoints);
        nextnbouridx = pos2ind(map,nbours);
        allPoints(nextindex).nbouridx = nextnbouridx;
        
        for i = 1:size(nextnbouridx,1)
            if allPoints(nextnbouridx(i)).visnovisobs == -1
                allPoints(nextnbouridx(i)).parent = nextindex;
                allPoints(nextnbouridx(i)).cost2reach = allPoints(nextindex).cost2reach + 1;
            else
                c2r = allPoints(nextnbouridx(i)).cost2reach;
                newc2r = allPoints(nextindex).cost2reach + 1;
                if newc2r < c2r
                    c2r = newc2r;
                end
            end
        end
        
        nextindex = getnextidx(allPoints);
    end
    allPoints(nextindex).visnovisobs = 1;
    path = getpath(allPoints,nextindex); 
end
toc