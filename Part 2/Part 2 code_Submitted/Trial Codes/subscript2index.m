function index = subscript2index(max,y,x,z)
% max = [maxy, maxx, maxz]

index = y + (x-1)*max(1) + (z-1)*max(1)*max(2);