function c2goal = heuristic(astar,goal,point)

if astar == 0
    c2goal = 0;
elseif astar == 1
    c2goal = norm(goal-point); 
else
    disp('Entered incorrect value');
end
