function [ijk] = pos2sub(map, pos)
% POS2SUB converts [x y z] position vectors into the subscripts [i, j, k]
% of the corresponding map voxels
%
% parameters:
%   pos - nx3 matrix of points; each row is an [x y z] position vector
%   map - map data structure loaded from a map textfile using load_map(...)
%   ijk - nx3 matrix of subscripts; each row is a set of subscripts
%         describing a voxel in the map
%
% NOTE: this function assumes pos is inside the map; bound checking must
%       be performed before calling

[jik] = floor((pos - map.bound_xyz(1:3))./map.res_xyz) + 1;
ijk = [jik(:,2) jik(:,1) jik(:,3)];