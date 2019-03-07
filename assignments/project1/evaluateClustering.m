function [] = evaluateClustering(X, Y, I)
   M = [X Y];
   k = [1:100];
   E = evalclusters(M,'kmeans','silhouette','klist',k)
   figure(2);
   plot(E)
end