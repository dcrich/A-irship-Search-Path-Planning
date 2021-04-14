
% figure(1)
% plot_flight_ceiling(2000)


% path = load('path.csv');
%
% % geoscatter(path(:,1),path(:,2), 'yo','filled')
% geoplot(path(:,1),path(:,2), 'k','LineWidth', 1)
% hold on
% geobasemap colorterrain
% % legend('Coordinates','Obstacles','Simulated Annealing', 'Pattern Search', 'A* search')
% % ax = gca;
% % ax.FontSize = 16;
% % hold off

color = [.9 .1 .1];
h = figure(2);
numfiles = 4900;
filename = 'SItestAnimated2.gif';
for i = 0:10:numfiles
    pathfile = sprintf('data/AltUpd/data0/path%d.csv', i);
    path2 = load(pathfile);
    geoplot(path2(:,1),path2(:,2),'m','LineWidth', 2)
    hold on
    geolimits([0 60],[-20 80])
    geobasemap colorterrain
    drawnow
    if mod(i,100) == 0
    frame = getframe(h);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    % Write to the GIF File
    if i == 0
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append');
    end
    %     hold on
    %     pause(0.01)
    %     color = color + [0,0.8/numfiles,0.8/numfiles];
    end
    if i == numfiles
        pathfile = sprintf('data/AltUpd/data0/fullpath.csv');
        path2 = load(pathfile);
        geoplot(path2(:,1),path2(:,2),'g','LineWidth', 2)
        geolimits([0 60],[-20 80])
        geobasemap colorterrain
        drawnow
        
        frame = getframe(h);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        % Write to the GIF File
        if i == 0
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
        else
            imwrite(imind,cm,filename,'gif','WriteMode','append');
        end
        pathfile = sprintf('data/AltUpd/data0/fullpath.csv');
        path2 = load(pathfile);
        geoplot(path2(:,1),path2(:,2),'g','LineWidth', 2)
        geolimits([0 60],[-20 80])
        geobasemap colorterrain
        drawnow
        
        frame = getframe(h);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        % Write to the GIF File
        if i == 0
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
        else
            imwrite(imind,cm,filename,'gif','WriteMode','append');
        end
        pathfile = sprintf('data/AltUpd/data0/fullpath.csv');
        path2 = load(pathfile);
        geoplot(path2(:,1),path2(:,2),'g','LineWidth', 2)
        geolimits([0 60],[-20 80])
        geobasemap colorterrain
        drawnow
        
        frame = getframe(h);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        % Write to the GIF File
        if i == 0
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
        else
            imwrite(imind,cm,filename,'gif','WriteMode','append');
        end
    end
end
% pathfile = sprintf('data/AltUpd/data7/fullpath.csv');
% path2 = load(pathfile);
% geoplot(path2(:,1),path2(:,2),'g','LineWidth', 2)
% geolimits([0 60],[-20 80])




% %%
% plot_flight_ceiling(2000)

function plot_flight_ceiling(alt)
if ~exist('elevationData','var')
    elevationData = readtable('LATLONSOURCE.txt');
end
north = elevationData.NORTH;
west = elevationData.WEST;
el = elevationData.MAX_ELEV;
LatLonEl = [north,west,el];
uniquedata = unique(LatLonEl,'rows');
LatLonElTable = table(uniquedata(:,1),uniquedata(:,2),uniquedata(:,3),...
    'VariableNames',{'LAT','LON','ElevationMeters'});


% -56 to 84
% -180 to 180
matrixlatLonEl = zeros(141,360,3);
j = 1;
for i = -180:179
    matrixlatLonEl(:,j,2) = i;
    j = j+1;
end

j = 1;
for i = 84:-1:-56
    matrixlatLonEl(j,:,1) = i;
    j = j+1;
end
for i = 1:360
    for j = 1:141
        indexForElevation = find(LatLonElTable.LON == matrixlatLonEl(1,i,2) ...
            & LatLonElTable.LAT == matrixlatLonEl(j,1,1));
        if ~isempty(indexForElevation)
            matrixlatLonEl(j,i,3) = max(LatLonElTable.ElevationMeters(indexForElevation));
        end
    end
end

[r1,c1] = find(matrixlatLonEl(:,:,3) > alt);
linInd5Lon = sub2ind(size(matrixlatLonEl),r1,c1,2*ones(size(r1)));
linInd5Lat = sub2ind(size(matrixlatLonEl),r1,c1,ones(size(r1)));
[r1,c1] = find(matrixlatLonEl(:,:,3) < 4000);
linInd1Lat = sub2ind(size(matrixlatLonEl),r1,c1,ones(size(r1)));
linInd1Lon = sub2ind(size(matrixlatLonEl),r1,c1,2*ones(size(r1)));

geoscatter(matrixlatLonEl(linInd1Lat), matrixlatLonEl(linInd1Lon),'k','filled','MarkerFaceAlpha',.4)
hold on
geoscatter(matrixlatLonEl(linInd5Lat), matrixlatLonEl(linInd5Lon),'r','filled','MarkerFaceAlpha',.9)


end