function nextindex = getnextidx(allPoints)

notvisited = find([allPoints.visnovisobs] == -1);
notvisitedcost = [allPoints(notvisited).totalcost];
indices = find(notvisitedcost == min(notvisitedcost));
nextindex = notvisited(indices(1));