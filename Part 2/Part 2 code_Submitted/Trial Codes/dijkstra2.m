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

astar = 0;
% All points defined as [y x z]
start = [0 0 0];
goal = [0 0 1];

map = load_map('map0.txt', 0.1,1,1);

startidx = pos2ind(map,start);
goalidx = pos2ind(map,goal);

boundary = map.bound_xyz;
grid = map.occgrid;
xyz_res = map.res_xyz;

maxy = size(grid,1);
maxx = size(grid,2);
maxz = size(grid,3);
totalidx = maxy*maxx*maxz;

% Create an array of indices that haven't yet been visited
novis_array = find(map.occgrid == 1);
% Create an array of indices that are within the obstacle
obstaclearray = find(map.occgrid == 0);

allPoints = struct;
for i = 1:totalidx
    allPoints(i).index = i;
    allPoints(i).cost2reach = inf;
    allPoints(i).point = ind2pos(map,i);
end

% Create a struct with points that haven't been visited
% NoVisPoints = allPoints;
NoVisPoints = rmfield(allPoints,'point');

point = start;
startidx = pos2ind(map,point);

% Eliminate point just visited from the not visited array as well as struct
s = find(novis_array == startidx);
novis_array(s) = [];

allPoints(startidx).cost2reach = 0;
% allPoints(startidx).c2goal = heuristic(astar);
allPoints(startidx).parent = -1;
neighbours = getneighbours(map,allPoints(startidx).point);
nbouridx = pos2ind(map,neighbours);
allPoints(startidx).nbouridx = nbouridx;
delidx = find([NoVisPoints.index] == startidx);
NoVisPoints(delidx) = [];

for i = 1:size(nbouridx,1)
    allPoints(nbouridx(i)).parent = startidx;
    allPoints(nbouridx(i)).cost2reach = allPoints(startidx).cost2reach + 1;
    x = find([NoVisPoints.index] == nbouridx(i));
    NoVisPoints(x).cost2reach = allPoints(nbouridx(i)).cost2reach;
end

nxtidx = find(min([NoVisPoints.cost2reach]));
nextidx = NoVisPoints(nxtidx).index;
while nextidx~=goalidx
    check = ~isempty(find([NoVisPoints.index] == nxtidx, 1));
    if check == 0
        nextnbours = getneighbours(map,allPoints(nextidx).point);
        nextnbouridx = pos2ind(map,nextnbours);
        allPoints(nextidx).nbouridx = nextnbouridx;
        delidx = find([NoVisPoints.index] == nextidx);
        NoVisPoints(delidx) = [];
        
        for i = 1:size(nextnbouridx,1)
            checknbour = ~isempty(find([NoVisPoints.index] == nextnbouridx, 1));
            if checknbour == 0
                allPoints(nextnbouridx(i)).parent = nextidx;
                allPoints(nextnbouridx(i)).cost2reach = allPoints(nextidx).cost2reach + 1;
                x = find([NoVisPoints.index] == nextnbouridx(i));
                NoVisPoints(x).cost2reach = allPoints(nextnbouridx(i)).cost2reach;
            else
                c2r = allPoints(nextidx).cost2reach + 1;
                if allPoints(nextnbouridx(i)).cost2reach > c2r
                    allPoints(nextnbouridx(i)).cost2reach = c2r;
                end
            end
        end
        nextidx = find(min([NoVisPoints.cost2reach]));
    else
        apindx = find([allPoints.index] == nextidx);
        nbourpts = allPoints(apindx).nbouridx;
        for i = 1:size(nbourpts,1)
            c2rfromnbours(i,1) = allPoints(nbourpts(i)).cost2reach + 1;
        end
        
        updateidx = find(min(c2rfromnbours));
        allPoints(apindx).cost2reach = c2rfromnbours(updateidx,1);
        allPoints(apindx).parent = nbourpts(updateidx);
        nextidx = find(min([NoVisPoints.cost2reach]));
        % If the point has already been visited, see its neighbours and the
        % cost 2 reach that point from all the neighbours. Save the minimum
        % cost, discard the rest. If any of the neighbours has not been
        % previously visited, it means that the point in consideration has
        % not been approached from that point. So just leave it as it is.
        %nextnbours = allPoints(apindx).nbouridx;
    end
    
end