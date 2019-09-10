function map = load_map(filename, res_xy, res_z, margin)
% LOAD_MAP load grid map from textfile (see map0.txt for an example)
%
% MAP = LOAD_MAP(filename, res_xy, res_z, margin) loads the environment
% defined in filename and creates a map struct with the following fields:
%   res_xyz   - 1x3 vector of the length (in meters) of each side of the
%               voxels in the grid
%   bound_xyz - 1x6 vector of the minimum (lower-left-front) and maximum
%               (upper-right-back) corners of the bounding box of the
%               environment stored as [x_min y_min z_min x_max y_max z_max]
%   blocks    - 1x9 vector of the minimum (lower-left-front) and maximum
%               (upper-right-back) corners of each cuboid obstacle along
%               with a RGB color triplet for visualization stored as:
%               [x_min y_min z_min x_max y_max z_max R G B]
%   margin    - the distance (in meters) from an obstacle at which a path
%               is considered to be in collision (NOTE: each obstacle is
%               inflated by margin on all sides before intersecting voxels
%               are marked as occupied in the grid)
%   occgrid   - a 3D matrix of voxels with value 0 if the area enclosed by
%               the voxel is free and value 1 if an obstacle intersects the
%               voxel (NOTE: the origin of each voxel is located at the
%               lower-left-front corner)
%
% parameters:
%   filename - text file containing map bounds and obstacles
%   res_xy   - grid resolution in x, y dimensions in meters
%   res_z    - grid resolution in z dimension in meters
%   margin   - the distance with which to inflate obstacles in the
%              resulting map to avoid collisions

if nargin < 4
  error('usage: map = load_map(filename, res_xy, res_z, margin)');
end

% parse text file
fid = fopen(filename);
lines = textscan(fid, '%s %f %f %f %f %f %f %f %f %f', 'CommentStyle', '#');
boundary = [];
blocks = [];
vals = [lines{2:10}];
types = lines{1};
for k = 1:length(types)
    type = types{k};
    if strcmp(type, 'boundary') == 1
        boundary = vals(k, 1:end-3);
    elseif strcmp(type, 'block') == 1
        blocks = [blocks; vals(k, :)];
    else
        error('Unrecognized type');
    end
end
fclose(fid);

% initialize map struct
map.res_xyz = [res_xy, res_xy, res_z];
map.bound_xyz = boundary;
map.blocks = blocks;
map.margin = margin;

% construct voxel grid
hwd = pos2sub(map,map.bound_xyz(4:6)); % map dimensions [height, width, depth]
map.occgrid = zeros(hwd);

% mark obstacles as occupied
map_sz = size(map.occgrid);
for i = 1:size(blocks,1)
  ijk_min = pos2sub(map, blocks(i,1:3) - margin*ones(1,3));
  ijk_min(ijk_min < 1) = 1;
  ijk_min(ijk_min > map_sz) = map_sz(ijk_min > map_sz);

  ijk_max = pos2sub(map, blocks(i,4:6) + margin*ones(1,3));
  ijk_max(ijk_max < 1) = 1;
  ijk_max(ijk_max > map_sz) = map_sz(ijk_max > map_sz);

  [I, J, K] = meshgrid(ijk_min(1):ijk_max(1), ijk_min(2):ijk_max(2),...
                     ijk_min(3):ijk_max(3));
  map.occgrid(sub2ind(map_sz,I(:),J(:),K(:))) = 1;
end

end