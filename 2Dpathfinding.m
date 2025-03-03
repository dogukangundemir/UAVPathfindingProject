
nmaxVert = 15;
pdwnSmple = 0.6; 
climbAltitude = 50; 
cruiseAltitude = 30; 
descendAltitude = 0; 
safetyBuffer = 5; 


filename = 'map.osm';  
osmData = xml2struct(filename);


buildings = extractBuildings(osmData);
roads = extractRoads(osmData);

startPoint = [-73.978602, 40.762387];  
endPoint = [-73.973034, 40.764288]; 



fprintf('Enter additional waypoints between start and end points:\n');
waypoints = [startPoint];
while true
    answer = input('Enter waypoint as [lon, lat] or "done" to finish: ', 's');
    if strcmpi(answer, 'done')
        break;
    end
    waypoint = str2num(answer); 
    if numel(waypoint) == 2
        waypoints = [waypoints; waypoint];
    else
        disp('Invalid input. Please enter in the format [lon, lat].');
    end
end
waypoints = [waypoints; endPoint];


roiBuildings = ComputeROI(buildings, startPoint, endPoint);


for i = 1:length(roiBuildings)
    roiBuildings(i).vertices = ReduceMapVertices(roiBuildings(i).vertices, nmaxVert, pdwnSmple);
end


for i = 1:length(roiBuildings)
    vertices = roiBuildings(i).vertices;
    height = roiBuildings(i).height + safetyBuffer;
    roiBuildings(i).geofence3D = [vertices, repmat(height, size(vertices, 1), 1)];
end


visibilityGraph = constructVisibilityGraph(waypoints, roiBuildings);


allPaths = findAllPaths(visibilityGraph, 1, size(waypoints, 1));


[path, totalCost] = dijkstra(visibilityGraph, 1, size(waypoints, 1));


if isempty(allPaths)
    error('No path found between start and end points.');
end

% climbing, cruising, and descending
uavPaths = cellfun(@(p) generateUavPath(waypoints, p, climbAltitude, cruiseAltitude, descendAltitude), allPaths, 'UniformOutput', false);
optimalUavPath = generateUavPath(waypoints, path, climbAltitude, cruiseAltitude, descendAltitude);



disp('All paths (waypoint indices):');
disp(allPaths);
disp('Optimal path (waypoint indices):');
disp(path);
disp('Total cost of optimal path:');
disp(totalCost);


figure;
hold on;



for i = 1:length(roiBuildings)
    geofence3D = roiBuildings(i).geofence3D;
    fill(geofence3D(:,1), geofence3D(:,2), 'r', 'FaceAlpha', 0.5);
end


for i = 1:length(roads)
    plot(roads{i}(:,1), roads{i}(:,2), 'g--');
end


colors = lines(length(uavPaths)); 
for i = 1:length(uavPaths)
    plot2DPath(uavPaths{i}, colors(i,:));
end

plot(startPoint(1), startPoint(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(endPoint(1), endPoint(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
plot(waypoints(:, 1), waypoints(:, 2), 'x', 'Color', [1, 0.5, 0], 'MarkerSize', 10, 'LineWidth', 2);

plot2DPath(optimalUavPath, 'k', 2);

hold off;
xlabel('Longitude');
ylabel('Latitude');
title('2D Map with UAV Paths');
grid on;

% Plot the optimal path
plotOptimalPath(waypoints, optimalUavPath, roiBuildings, roads);
plotOsmMap(buildings, roads, waypoints);


dangerousZone = createRandomDangerousZone(roiBuildings);

% **Important**: Include the dangerous zone in the list of obstacles
roiBuildingsWithDanger = [roiBuildings; dangerousZone];

% Construct the updated visibility graph that includes the dangerous zone
visibilityGraphUpdated = constructVisibilityGraph(waypoints, roiBuildingsWithDanger);

% Recalculate the path to avoid the dangerous zone
[pathUpdated, totalCostUpdated] = dijkstra(visibilityGraphUpdated, 1, size(waypoints, 1));
optimalUavPathUpdated = generateUavPath(waypoints, pathUpdated, climbAltitude, cruiseAltitude, descendAltitude);

% Plot the new optimal path avoiding the dangerous zone
plotMapWithDangerZone(waypoints, optimalUavPathUpdated, buildings, roads, dangerousZone);






function buildings = extractBuildings(osmData)
    
    buildings = [];
    ways = osmData.osm.way;
    for i = 1:length(ways)
        if isfield(ways{i}, 'tag')
            tags = ways{i}.tag;
            isBuilding = false;
            if iscell(tags)
                for j = 1:length(tags)
                    if isfield(tags{j}.Attributes, 'k') && strcmp(tags{j}.Attributes.k, 'building')
                        isBuilding = true;
                        break;
                    end
                end
            elseif isstruct(tags)
                if isfield(tags.Attributes, 'k') && strcmp(tags.Attributes.k, 'building')
                    isBuilding = true;
                end
            end

            if isBuilding
                nodeRefs = ways{i}.nd;
                vertices = [];
                for j = 1:length(nodeRefs)
                    ref = nodeRefs{j}.Attributes.ref;
                    node = findNode(osmData, ref);
                    vertices = [vertices; str2double(node.Attributes.lon), str2double(node.Attributes.lat)];
                end
                height = 10; % Default height
                buildings = [buildings; struct('vertices', vertices, 'height', height)];
            end
        end
    end
end

function roads = extractRoads(osmData)
    roads = [];
    ways = osmData.osm.way;
    for i = 1:length(ways)
        if isfield(ways{i}, 'tag')
            tags = ways{i}.tag;
            isRoad = false;
            if iscell(tags)
                for j = 1:length(tags)
                    if isfield(tags{j}.Attributes, 'k') && strcmp(tags{j}.Attributes.k, 'highway')
                        isRoad = true;
                        break;
                    end
                end
            elseif isstruct(tags)
                if isfield(tags.Attributes, 'k') && strcmp(tags.Attributes.k, 'highway')
                    isRoad = true;
                end
            end

            if isRoad
                nodeRefs = ways{i}.nd;
                roadVertices = [];
                for j = 1:length(nodeRefs)
                    ref = nodeRefs{j}.Attributes.ref;
                    node = findNode(osmData, ref);
                    roadVertices = [roadVertices; str2double(node.Attributes.lon), str2double(node.Attributes.lat)];
                end
                roads = [roads; {roadVertices}];
            end
        end
    end
end

function node = findNode(osmData, ref)
    % Finds a node with a given reference ID in OSM data
    nodes = osmData.osm.node;
    for i = 1:length(nodes)
        if strcmp(nodes{i}.Attributes.id, ref)
            node = nodes{i};
            return;
        end
    end
    error('Node not found');
end

function reducedVertices = ReduceMapVertices(vertices, nmaxVert, pdwnSmple)
    % Reduces the number of vertices in a polygon
    if size(vertices, 1) > nmaxVert
        % Downsample the vertices
        step = round(1 / pdwnSmple);
        reducedVertices = vertices(1:step:end, :);
        if size(reducedVertices, 1) < 3
            reducedVertices = vertices; % Ensure we still have a valid polygon
        end
    else
        reducedVertices = vertices;
    end
end

function roiBuildings = ComputeROI(buildings, startPoint, endPoint)
    % Computes buildings within the region of interest (ROI)
    hullPoints = [startPoint; endPoint];

    % Find buildings within the convex hull
    inHull = false(size(buildings));
    for i = 1:length(buildings)
        buildingVertices = buildings(i).vertices;
        inHull(i) = any(inpolygon(buildingVertices(:, 1), buildingVertices(:, 2), hullPoints(:, 1), hullPoints(:, 2)));
    end

    % Filter buildings to include only those in the ROI
    roiBuildings = buildings(inHull);
end

function intersects = checkIntersection(p1, p2, obstacles)
    % Checks if a line segment intersects with any obstacle edges
    intersects = false;
    for i = 1:length(obstacles)
        vertices = obstacles(i).vertices;
        numVertices = size(vertices, 1);
        for j = 1:numVertices
            p3 = vertices(j, :);
            p4 = vertices(mod(j, numVertices) + 1, :);
            if segmentsIntersect(p1, p2, p3, p4)
                intersects = true;
                return;
            end
        end
    end
end

function intersect = segmentsIntersect(p1, p2, p3, p4)
    d1 = direction(p3, p4, p1);
    d2 = direction(p3, p4, p2);
    d3 = direction(p1, p2, p3);
    d4 = direction(p1, p2, p4);
    intersect = (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ...
                 ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) || ...
                (d1 == 0 && onSegment(p3, p4, p1)) || ...
                (d2 == 0 && onSegment(p3, p4, p2)) || ...
                (d3 == 0 && onSegment(p1, p2, p3)) || ...
                (d4 == 0 && onSegment(p1, p2, p4));
end

function d = direction(p1, p2, p3)
    d = (p3(1) - p1(1)) * (p2(2) - p1(2)) - (p2(1) - p1(1)) * (p3(2) - p1(2));
end

function onSeg = onSegment(p1, p2, p)

    onSeg = min(p1(1), p2(1)) <= p(1) && p(1) <= max(p1(1), p2(1)) && ...
            min(p1(2), p2(2)) <= p(2) && p(2) <= max(p1(2), p2(2));
end

% visibility graph
function visibilityGraph = constructVisibilityGraph(waypoints, obstacles)
    
    n = size(waypoints, 1);
    visibilityGraph = inf(n);
    for i = 1:n
        for j = 1:n
            if i ~= j
                if ~checkIntersection(waypoints(i, :), waypoints(j, :), obstacles)
                    distance = norm(waypoints(i, :) - waypoints(j, :));
                    visibilityGraph(i, j) = distance;
                end
            end
        end
    end
end

function [path, totalCost] = dijkstra(visibilityGraph, startIdx, endIdx)
    % optimal shortest path finding dijkstra 
    n = size(visibilityGraph, 1);
    unvisited = 1:n;
    distance = inf(1, n);
    previous = nan(1, n);
    distance(startIdx) = 0;

    while ~isempty(unvisited)
        [~, idx] = min(distance(unvisited));
        current = unvisited(idx);
        if current == endIdx
            break;
        end
        unvisited(idx) = [];

        neighbors = find(visibilityGraph(current, :) < inf);
        for neighbor = neighbors
            alt = distance(current) + visibilityGraph(current, neighbor);
            if alt < distance(neighbor)
                distance(neighbor) = alt;
                previous(neighbor) = current;
            end
        end
    end

    % Path construction
    path = [];
    totalCost = distance(endIdx);
    u = endIdx;
    while ~isnan(u)
        path = [u, path];
        u = previous(u);
    end
end

function uavPath = generateUavPath(waypoints, path, climbAltitude, cruiseAltitude, descendAltitude)
    % genearting path for climbing descending and cruising
    uavPath = [];
    fprintf('Path length is %d : \n',length(path));
    for i = 1:length(path)
        waypoint = waypoints(path(i), :);
        if i == 1 
            uavPath = [uavPath; waypoint, climbAltitude];
        elseif i == length(path) 
            uavPath = [uavPath; waypoint, descendAltitude];
        else
            uavPath = [uavPath; waypoint, cruiseAltitude];
        end
    end
end

function plot2DPath(uavPath, color, lineWidth)
  
    if nargin < 3
        lineWidth = 1;
    end
    
    for i = 1:size(uavPath, 1)-1
        p1 = uavPath(i, :);
        p2 =uavPath(i+1, :);

    if p1(3) > 30  % Climbing
        lineColor = [0 ,0 ,0];
    elseif p1(3) > 10 && p1(3)< 30 % Cruising
        lineColor = [1, 1, 0]; 
    else % Descending
        lineColor = [1, 0, 0];
    end
    
    plot([p1(1), p2(1)], [p1(2), p2(2)], 'Color', lineColor, 'LineWidth', lineWidth);
    if i < size(uavPath, 1)-1
        p3 = uavPath(i+2, :);
        mid1 = (p1 + p2) / 2;
        mid2 = (p2 + p3) / 2;
   
        t = linspace(0, 1, 100);
        x = (1-t).^2 * mid1(1) + 2*(1-t).*t * p2(1) + t.^2 * mid2(1);
        y = (1-t).^2 * mid1(2) + 2*(1-t).*t * p2(2) + t.^2 * mid2(2);
        plot(x, y, 'Color', lineColor, 'LineWidth', lineWidth);
    end
end

end

function allPaths = findAllPaths(graph, startNode, endNode)
% dfs to find all paths
allPaths = {};
stack = {startNode};
pathStack = {startNode};

while ~isempty(stack)
    currentNode = stack{end};
    currentPath = pathStack{end};
    stack(end) = [];
    pathStack(end) = [];

    if currentNode == endNode
        allPaths = [allPaths; {currentPath}];
    else
        neighbors = find(graph(currentNode, :) < inf);
        for i = 1:length(neighbors)
            neighbor = neighbors(i);
            if ~ismember(neighbor, currentPath)
                stack = [stack; neighbor];
                pathStack = [pathStack; [currentPath, neighbor]];
            end
        end
    end
end

end

function plotOsmMap(buildings, roads, waypoints)
    figure;
    hold on;
    
    % Plot buildings
    for i = 1:length(buildings)
        fill(buildings(i).vertices(:,1), buildings(i).vertices(:,2), 'r', 'FaceAlpha', 0.5);
    end

    % Plot roads
    for i = 1:length(roads)
        plot(roads{i}(:,1), roads{i}(:,2), 'g--');
    end

    % Plot waypoints as orange crosses
    plot(waypoints(:, 1), waypoints(:, 2), 'x', 'Color', [1, 0.5, 0], 'MarkerSize', 10, 'LineWidth', 2);
    title('OSM Data');
    xlabel('Longitude');
    ylabel('Latitude');
    grid on;
    hold off;
end

function plotOptimalPath(waypoints, uavPath, buildings, roads)
    figure;
    hold on;

    % Plot buildings
    for i = 1:length(buildings)
        fill(buildings(i).vertices(:,1), buildings(i).vertices(:,2), 'r', 'FaceAlpha', 0.5);
    end

    % Plot roads
    for i = 1:length(roads)
        plot(roads{i}(:,1), roads{i}(:,2), 'g--');
    end

    % Plot waypoints as orange crosses
    plot(waypoints(:, 1), waypoints(:, 2), 'x', 'Color', [1, 0.5, 0], 'MarkerSize', 10, 'LineWidth', 2);

    % Plot path with altitude changes
    for i = 1:size(uavPath, 1)-1
        p1 = uavPath(i, :);
        p2 = uavPath(i+1, :);
        
        % Determine altitude phase for coloring
        if p1(3) < p2(3)
            % Climbing
            lineColor = [1, 0, 0]; % Red
        elseif p1(3) > p2(3)
            % Descending
            lineColor = [0, 0, 1]; % Blue
        else
            % Cruising
            lineColor = [0, 1, 0]; % Green
        end
        
        plot([p1(1), p2(1)], [p1(2), p2(2)], 'Color', lineColor, 'LineWidth', 2);
        
        % Altitude annotation
        text(p1(1), p1(2), sprintf('%.1fm', p1(3)), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    end

    % Final waypoint altitude annotation
    text(uavPath(end, 1), uavPath(end, 2), sprintf('%.1fm', uavPath(end, 3)), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    
    title('Optimal Path');
    xlabel('Longitude');
    ylabel('Latitude');
    grid on;
    hold off;
end

function dangerousZone = createRandomDangerousZone(roiBuildings)
    % Create a random dangerous zone within the ROI
    minX = min(cellfun(@(v) min(v(:,1)), {roiBuildings.vertices}));
    maxX = max(cellfun(@(v) max(v(:,1)), {roiBuildings.vertices}));
    minY = min(cellfun(@(v) min(v(:,2)), {roiBuildings.vertices}));
    maxY = max(cellfun(@(v) max(v(:,2)), {roiBuildings.vertices}));
    
    centerX = -73.9735;
    centerY = 40.7636;
    
    width = (maxX - minX) * 0.1;
    height = (maxY - minY) * 0.1;
    
    vertices = [
        centerX - width/2, centerY - height/2;
        centerX + width/2, centerY - height/2;
        centerX + width/2, centerY + height/2;
        centerX - width/2, centerY + height/2
    ];
    
    % Create dangerousZone structure with the same fields as roiBuildings
    dangerousZone.vertices = vertices;
    dangerousZone.height = 10; % Arbitrary height for the dangerous zone
    dangerousZone.geofence3D = [vertices, repmat(dangerousZone.height, size(vertices, 1), 1)];
end

function plotMapWithDangerZone(waypoints, uavPath, buildings, roads, dangerousZone)
    figure;
    hold on;

    % Plot buildings
    for i = 1:length(buildings)
        fill(buildings(i).vertices(:,1), buildings(i).vertices(:,2), 'r', 'FaceAlpha', 0.5);
    end

    % Plot roads
    for i = 1:length(roads)
        plot(roads{i}(:,1), roads{i}(:,2), 'g--');
    end

    % Plot waypoints as orange crosses
    plot(waypoints(:, 1), waypoints(:, 2), 'x', 'Color', [1, 0.5, 0], 'MarkerSize', 10, 'LineWidth', 2);

    % Plot dangerous zone
    fill(dangerousZone.vertices(:,1), dangerousZone.vertices(:,2), 'k', 'FaceAlpha', 0.8);

    % Plot path with altitude changes
    for i = 1:size(uavPath, 1)-1
        p1 = uavPath(i, :);
        p2 = uavPath(i+1, :);
        
        % Determine altitude phase for coloring
        if p1(3) < p2(3)
            % Climbing
            lineColor = [1, 0, 0]; % Red
        elseif p1(3) > p2(3)
            % Descending
            lineColor = [0, 0, 1]; % Blue
        else
            % Cruising
            lineColor = [0, 1, 0]; % Green
        end
        
        plot([p1(1), p2(1)], [p1(2), p2(2)], 'Color', lineColor, 'LineWidth', 2);
        
        % Altitude annotation
        text(p1(1), p1(2), sprintf('%.1fm', p1(3)), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    end

    % Final waypoint altitude annotation
    text(uavPath(end, 1), uavPath(end, 2), sprintf('%.1fm', uavPath(end, 3)), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    
    title('Map with Dangerous Zone and UAV Path');
    xlabel('Longitude');
    ylabel('Latitude');
    grid on;
    hold off;
end
