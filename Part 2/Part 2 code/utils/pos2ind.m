function ind = pos2ind(map,pos)
% POS2IND converts [x y z] position vectors to the linear matrix indices of
% the corresponding map voxels
%
% parameters:
%   map - map data structure loaded from a map textfile using load_map(...)
%   pos - nx3 matrix of points; each row is an [x y z] position vector
%   ind - nx1 vector of linear matrix indices
%
% NOTE: this function assumes the given points are within bounds of the
%       map matrix; bound checking must be performed before calling

ijk = pos2sub(map, pos);
ind = sub2ind(size(map.occgrid),ijk(:,1), ijk(:,2), ijk(:,3));