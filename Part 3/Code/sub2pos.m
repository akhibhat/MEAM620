function [pos] = sub2pos(map, ijk)
% SUB2POS converts an [i j k] vector of subscripts to the world [x y z]
% position of the corresponding map voxel
%
% parameters:
%   ijk - nx3 matrix of subscripts; each row is a set of subscripts
%         describing a voxel in the map
%   map - map data structure loaded from a map textfile using load_map(...)
%   pos - nx3 matrix of points; each row is an [x y z] position vector
%
% NOTE: this function assumes the give subscripts are within bounds of the
%       map matrix; bound checking must be performed before calling

pos = ([ijk(:,2) ijk(:,1) ijk(:,3)] - 1).*map.res_xyz + map.bound_xyz(1:3);