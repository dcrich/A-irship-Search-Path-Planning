path = load('path.csv')
hold on
geoscatter(path(:,1),path(:,2), 'ko','filled')
geobasemap colorterrain
