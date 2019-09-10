function plot_path(map, path)
% PLOT_PATH visualize a path through a voxel grid
%
%   PLOT_PATH(map) creates a 3D visualization of the map and obstacles
%
%   PLOT_PATH(map, path) creates a 3D visualization of the map and
%   obstacles and plots a path through the environment
%
% parameters:
%   map  - map struct loaded from a map textfile using load_map(..)
%   path - nx3 matrix where each row corresponds to the (x, y, z)
%          coordinates of one point along the path

ax = gca;
hold on;

% plot obstacles from map textfile
for i = 1:size(map.blocks,1)
  vert0 = map.blocks(i,1:3);
  vert1 = map.blocks(i,4:6);
  dx = [vert1(1) - vert0(1), 0, 0];
  dy = [0, vert1(2) - vert0(2), 0];
  dz = [0, 0, vert1(3) - vert0(3)];
  face = zeros(4,3,6);
  face(:,:,1) = [vert0; vert0 + dx; vert0 + dx + dy; vert0 + dy];
  face(:,:,2) = [vert0; vert0 + dx; vert0 + dx + dz; vert0 + dz];
  face(:,:,3) = [vert0; vert0 + dy; vert0 + dy + dz; vert0 + dz];
  face(:,:,4) = face(:,:,1) + repmat(dz,[4 1]);
  face(:,:,5) = face(:,:,2) + repmat(dy,[4 1]);
  face(:,:,6) = face(:,:,3) + repmat(dx,[4 1]);
  vertx = reshape(face(:,1,:), [4 6 1]);
  verty = reshape(face(:,2,:), [4 6 1]);
  vertz = reshape(face(:,3,:), [4 6 1]);

  fill3(vertx, verty, vertz, map.blocks(i,7:9)/255);
end

% make figure
axis equal
bounds = map.bound_xyz;
res = map.res_xyz;
axis([bounds(1) bounds(4) bounds(2) bounds(5) bounds(3) bounds(6)]);
ax.XAxis.MinorTickValues = bounds(1):res(1):bounds(4);
ax.YAxis.MinorTickValues = bounds(2):res(2):bounds(5);
ax.ZAxis.MinorTickValues = bounds(3):res(3):bounds(6);
ax.MinorGridLineStyle = '-';
grid minor

if nargin > 1
  plot3(path(:,1), path(:,2), path(:,3), 'k', 'LineWidth', 2);
  plot3(path(1,1), path(1,2), path(1,3), 'go', 'MarkerSize', 12, 'LineWidth', 2);
  plot3(path(end,1), path(end,2), path(end,3), 'mx', 'MarkerSize', 12, 'LineWidth', 2);
end

xlabel('x (meters)');
ylabel('y (meters)');
zlabel('z (meters)');

view(3);

end
