function c2goal = heuristic(astar,goal,point)

if astar == 0
    c2goal = 0;
elseif astar == 1
    c2goal = sqrt((goal(1)-point(1))^2 + (goal(2)-point(2))^2 + (goal(3)-point(3))^2);  
else
    disp('Entered incorrect value');
end
