clear; clc;


altSet = 1;

lowC = 0.1;
highC = 0.9;
red = [0.9*highC,lowC,0.5*highC];
blue = [lowC,1.1*lowC,highC];
green = [lowC,highC,1.1*lowC];


if ~exist('elevationLatLon','var')
    elevationLatLon = load('elevation3D.txt');
end


for testSet = [0,1,2,3,4]
    if altSet == 1
        pathfileW0 = sprintf('data/AltUpdKM/data%d/fullpath.csv', testSet*2+1);
        pathfileW1 = sprintf('data/AltUpdKM/data%d/fullpath.csv', testSet*2);
        cruiseAlt = 1000;
        flightCeiling = 2000;
        %         cruiseAlt = 2100;
        %         flightCeiling = 3300;
    else
        pathfileW0 = sprintf('data/NoAlt/data%d/fullpath.csv', testSet*2+1);
        pathfileW1 = sprintf('data/NoAlt/data%d/fullpath.csv', testSet*2);
        cruiseAlt = 9000;
        flightCeiling = 9000;
    end
    
    if testSet == 0
        % SPAIN to India
        startLatLon = [41, -9];
        endLatLon = [11,78];
        fprintf('Spain to India\n')
    elseif testSet == 1
        % AK to Panama
        startLatLon = [65, -151];
        endLatLon = [9,-80];
        fprintf('Alaska to Panama\n')
    elseif testSet == 2
        startLatLon = [-35, 21];
        endLatLon = [-56,-70];
        fprintf('South Africa to Tierra del Fuego \n')
    elseif testSet == 3
        startLatLon = [-5,-83];
        endLatLon = [-5,-76] ;
        fprintf('Peru Coast to Peru Inland\n')
    elseif testSet == 4
        startLatLon = [42,-71];
        endLatLon = [59,11];
        fprintf('Boston to Oslo \n')
    end
    fprintf('Cruise Altitude: %dm\n\n',cruiseAlt)
    
    
    
%         plot_flight_ceiling(flightCeiling,elevationLatLon)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%% Path without altitude constraints - GC Distance %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % // calculate distance in meters
    gcDistance(testSet+1) = distance_calc(startLatLon,endLatLon);
    [lat1,lon1,lat2,lon2] = convert_to_radians(startLatLon, endLatLon);
    
    fprintf('Great Circle: %.0f\n',gcDistance(testSet+1))
    if lon1 == lon2
        lon = lon1;
        lon = lon || lon2;
        lat = lat1;
        lat = lat || lat2;
    else
        numWaypoints = abs(startLatLon(2)-endLatLon(2));
        % create lon waypoints
        lon = linspace( lon1, lon2, numWaypoints );
        % calculate lat waypoints
        lat = zeros(1, numWaypoints);
        for i = 1:numWaypoints
            lat(i) = atan((sin( lat1 ) * cos( lat2 ) * sin( lon(i) - lon2 ) - sin( lat2 )  ...
                * cos( lat1 ) * sin( lon(i) - lon1 )) / (cos( lat1 ) * cos( lat2 ) *sin( lon1 - lon2 )));
        end
    end
    [lat,lon] = convert_to_degrees(lat,lon);
    disclat = round(lat);
    disclon = round(lon);
    gcDistanceDisc(testSet+1) = 0;
    gcDistanceAlt(testSet+1) = 0;
    for i=1:length(disclon)-1
        if disclat(i) < -56
            disclat(i) = -56;
        elseif disclat(i+1) < -56
            disclat(i+1) = -56;
        end
        
        gcDistanceDisc(testSet+1) = gcDistanceDisc(testSet+1) + distance_calc([disclat(i),disclon(i)], [disclat(i+1),disclon(i+1)]);
        gcDistanceAlt(testSet+1) = gcDistanceAlt(testSet+1) + ...
            total_dist([disclat(i),disclon(i)],[disclat(i+1),disclon(i+1)],elevationLatLon, cruiseAlt);
    end
    fprintf('Discretized Great Circle Ground: %.0f\n',gcDistanceDisc(testSet+1))
    fprintf('Discretized Great Circle Total: %.0f\n',gcDistanceAlt(testSet+1))
    geoplot(lat,lon,'Color','w', 'LineWidth',2)
    % geoplot(disclat,disclon,'Color','k', 'LineWidth',2)
    % geoscatter(disclat,disclon,'k', 'filled')
    hold on
    geobasemap colorterrain
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%% Path favoring algorithm speed:  %%%%%%
    %%%%%% low weight distance traveled,   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%  high weight distance remaining %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    path = load(pathfileW0);
    latW0 = path(:,1);
    lonW0 = path(:,2);
    
    
    aStarHeuristicGroundDistance(testSet+1) = 0;
    aStarHeuristicTotalDistance(testSet+1) = 0;
    for i=1:length(latW0)-1
        aStarHeuristicGroundDistance(testSet+1) = aStarHeuristicGroundDistance(testSet+1) + distance_calc([latW0(i),lonW0(i)], [latW0(i+1),lonW0(i+1)]);
        aStarHeuristicTotalDistance(testSet+1) = aStarHeuristicTotalDistance(testSet+1) + ...
            total_dist([latW0(i),lonW0(i)],[latW0(i+1),lonW0(i+1)],elevationLatLon, cruiseAlt);
        legDistHeur(i) = total_dist([latW0(i),lonW0(i)],[latW0(i+1),lonW0(i+1)],elevationLatLon, cruiseAlt);
    end
    fprintf('Weighted A* Ground: %.0f\n',aStarHeuristicGroundDistance(testSet+1))
    fprintf('Weighted A* Total: %.0f\n',aStarHeuristicTotalDistance(testSet+1))
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%% Path favoring better distance traveled: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%% equal weighting                         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    path = load(pathfileW1);
    latW1 = path(:,1);
    lonW1 = path(:,2);
    %     geoplot(latW1,lonW1,'Color','c','LineWidth', 2)
    
    aStarGroundDistance(testSet+1) = 0;
    aStarTotalDistance(testSet+1) = 0;
    for i=1:length(latW1)-1
        aStarGroundDistance(testSet+1) = aStarGroundDistance(testSet+1) + distance_calc([latW1(i),lonW1(i)], [latW1(i+1),lonW1(i+1)]);
        aStarTotalDistance(testSet+1) = aStarTotalDistance(testSet+1) + total_dist([latW1(i),lonW1(i)],[latW1(i+1),lonW1(i+1)],elevationLatLon, cruiseAlt);
    end
    fprintf('Unweighted A* Ground: %.0f\n',aStarGroundDistance(testSet+1))
    fprintf('Unweighted A* Total: %.0f\n',aStarTotalDistance(testSet+1))
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%% Matlab A* Setup %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elevationMarix = zeros(size(elevationLatLon));
    sizeElevationMat = size(elevationLatLon);
    for i = 1:sizeElevationMat(1)
        for j = 1: sizeElevationMat(2)
            if elevationLatLon(i,j) >= flightCeiling
                elevationMarix(i,j) = 1;
            elseif elevationLatLon(i,j) >= cruiseAlt && elevationLatLon(i,j) < flightCeiling
                elevationMarix(i,j) = elevationLatLon(i,j)/flightCeiling;
            end
        end
    end
    start = [85 - startLatLon(1),181 + startLatLon(2)];
    goal = [85 - endLatLon(1),181 + endLatLon(2)];
    elevationMarix(start(1),start(2)) = 0;
    elevationMarix(goal(1),goal(2)) = 0;
    map = occupancyMap(elevationMarix);
    planner = plannerAStarGrid(map);
    matlabAstarPath = plan(planner,start,goal);
    %%%%******* convert from path to lat,lon
    latM = 85 - matlabAstarPath(:,1);
    lonM  = matlabAstarPath(:,2) - 181;
    matGroundDistance(testSet+1) = 0;
    matTotalDistance(testSet+1) = 0;
    for i=1:length(latM)-1
        matGroundDistance(testSet+1) = matGroundDistance(testSet+1) + distance_calc([latM(i),lonM(i)], [latM(i+1),lonM(i+1)]);
        matTotalDistance(testSet+1) = matTotalDistance(testSet+1) + total_dist([latM(i),lonM(i)],[latM(i+1),lonM(i+1)],elevationLatLon, cruiseAlt);
        legDistMat(i) = total_dist([latM(i),lonM(i)],[latM(i+1),lonM(i+1)],elevationLatLon, cruiseAlt);
    end
    geoplot(latM,lonM,'k','LineWidth', 2)
    fprintf('Matlab A* Ground: %.0f\n',matGroundDistance(testSet+1))
    fprintf('Matlab A* Total: %.0f\n\n\n',matTotalDistance(testSet+1))
    
    geoplot(latW1,lonW1,'Color','m','LineWidth', 2)
    %Plot start and end points
    geoscatter(startLatLon(1),startLatLon(2),'filled','MarkerFaceColor', green,'MarkerEdgeColor','k')
    geoscatter(endLatLon(1),endLatLon(2),'filled','MarkerFaceColor', red,'MarkerEdgeColor','k')
    % hold off
    geolimits([0 60],[-20 80])
    
    ax = gca;
    ax.FontSize = 16;
    legend('Obstacles','Shortest Distance','Matlab A*','A*','Color',[0.8 0.8 0.8])
    %%%%%%%%% ERROR DATA %%%%%%%%%
    
    avgerrorMat(testSet+1) = (-matTotalDistance(testSet+1) + gcDistanceAlt(testSet+1)) / gcDistanceAlt(testSet+1);
    avgerrorA(testSet+1) = (-aStarTotalDistance(testSet+1) + gcDistanceAlt(testSet+1)) / gcDistanceAlt(testSet+1);
    AVGERRORMA(testSet+1) = (-aStarTotalDistance(testSet+1) + matTotalDistance(testSet+1)) / matTotalDistance(testSet+1);
    lengthlatM(testSet+1) = length(latM)
    lengthlatW1(testSet+1) = length(latW1)
    avgerrorMat(testSet+1) = (-matTotalDistance(testSet+1) );
    avgerrorA(testSet+1) = (-aStarTotalDistance(testSet+1) );
end
hold off
mean(avgerrorMat)
mean(avgerrorA)
% mean(AVGERRORMA)
% mean(lengthlatW1)
% mean(lengthlatM)

% figure(2)
% axesm('MapProjection','eqdconic')
% plotm(latW1,lonW1,'Color',blue)
% hold on
% plotm(latW0,lonW0,'Color',red)
% plotm(latM,lonM,'y')
% plotm(lat,lon,'k')
% hold off

% figure(3)
% show(planner)
%%
% totalDistanceEuc = total_dist([44,-8],[45,-7],elevationLatLon, cruiseAlt)
% totalDistMan = total_dist([44,-8],[45,-8],elevationLatLon, cruiseAlt) + total_dist([45,-8],[45,1],elevationLatLon, cruiseAlt)
for i = 0:70
    totalDistanceEuc(i+1) = distance_calc([i,0],[i+1,1]);
    totalDistMan(i+1) = distance_calc([i,0],[i+1,0]) + distance_calc([i+1,0],[i+1,1]);
end

meanManEucDiff = mean(totalDistMan - totalDistanceEuc)
maxDiff = max(totalDistMan) - max(totalDistanceEuc)
minDiff = min(totalDistMan) - min(totalDistanceEuc)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%    FUNCTIONS    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function distance = distance_calc(coordinate1, coordinate2)
lat1 = coordinate1(1) * pi / 180;
lat2 = coordinate2(1) * pi / 180;
lon1 = coordinate1(2) * pi / 180;
lon2 = coordinate2(2) * pi / 180;
distance = 6371 * asin( sqrt( (sin( (lat1 - lat2) / 2 )) ^ 2 + cos( lat1 ) * cos( lat2 ) * (sin( (lon1 - lon2) / 2 )) ^ 2 ) );
end

function totalDistance = total_dist(coordinate1,coordinate2,elevationData, cruiseAlt)
lat1 = coordinate1(1) * pi / 180;
lat2 = coordinate2(1) * pi / 180;
lon1 = coordinate1(2) * pi / 180;
lon2 = coordinate2(2) * pi / 180;
gDistance = 6371 * asin( sqrt( (sin( (lat1 - lat2) / 2 )) ^ 2 + cos( lat1 ) * cos( lat2 ) * (sin( (lon1 - lon2) / 2 )) ^ 2 ) );

lat1 = 85-coordinate1(1);
lat2 = 85-coordinate2(1);
lon1 = 181+coordinate1(2);
lon2 = 181+coordinate2(2);
elevation1 = elevationData(lat1,lon1);
elevation2 = elevationData(lat2,lon2);
elevationChange = elevation2 - elevation1;
if elevationChange > 0 && elevation1 > cruiseAlt && elevation2 > cruiseAlt
    elevDistance = elevationChange;
elseif elevationChange > 0 && elevation2 > cruiseAlt && elevation1 < cruiseAlt
    elevDistance = elevation2 - cruiseAlt;
else
    elevDistance = 0;
end
totalDistance = sqrt((elevDistance/1000)^2 + gDistance^2);
end

function [lat1,lon1,lat2,lon2] = convert_to_radians(coordinate1, coordinate2)
lat1 = coordinate1(1) * pi / 180;
lat2 = coordinate2(1) * pi / 180;
lon1 = coordinate1(2) * pi / 180;
lon2 = coordinate2(2) * pi / 180;
end

function [lat,lon] = convert_to_degrees(latArrray,lonArray)
lat = (180 / pi) * latArrray;
lon = (180 / pi) * lonArray;
end


function plot_flight_ceiling(alt,elevationLatLon)
% if ~exist('matrixlatLonEl','var')
%     elevationLatLon = load('elevation3D.txt');
% end

[r1,c1] = find(elevationLatLon > alt);
linInd5Lon = c1-181;
linInd5Lat = 85-r1;
[r1,c1] = find(elevationLatLon < alt);
% linInd1Lon = c1-181;
% linInd1Lat = 85-r1;

% geoscatter((linInd1Lat), (linInd1Lon),'k','filled','MarkerFaceAlpha',.1)
% hold on
geoscatter((linInd5Lat), (linInd5Lon),'r^','filled','MarkerFaceAlpha',.4)
geobasemap colorterrain
hold on
end