function pos = ind2pos(map,ind)
% IND2POS converts linear matrix indices to the [x y z] positions of the
% corresponding map voxel
%
% parameters:
%   map - map data structure loaded from a map textfile using load_map(...)
%   ind - nx1 vector of linear matrix indices
%   pos - nx3 matrix of points; each row is an [x y z] position vector
%
% NOTE: this function assumes the given indices are within bounds of the
%       map matrix; bound checking must be performed before calling

[I,J,K] = ind2sub(size(map.occgrid), ind);
pos = sub2pos(map, [I(:) J(:) K(:)]);