% Main Script

% User-defined parameters
nmaxVert = 15;
pdwnSmple = 0.6; 
climbAltitude = 50; 
cruiseAltitude = 30; 
descendAltitude = 0; 
safetyBuffer = 5; 

delta_vehicle = 5; % Safety buffer around vehicle in meters
delta_building = 10; % Safety buffer around buildings in meters
delta_sb = delta_vehicle + delta_building; % Total safety buffer

% Weighted cost function parameters
alpha = 1; % Weight for distance
beta = 1; % Weight for power consumption
gamma = 1; % Weight for time delay

% Load OSM data
filename = 'map.osm';  
osmData = xml2struct(filename);

% Extract detailed OSM data
buildings = extractBuildings(osmData);
roads = extractRoads(osmData);

% Define waypoints
startPoint = [-73.978602, 40.762387];  
endPoint = [-73.973034, 40.764288];  

% Get dynamic waypoints from the user
fprintf('Enter additional waypoints between start and end points:\n');
waypoints = [startPoint];
while true
    answer = input('Enter waypoint as [lon, lat] or "done" to finish: ', 's');
    if strcmpi(answer, 'done')
        break;
    end
    waypoint = str2num(answer); %#ok<ST2NM>
    if numel(waypoint) == 2
        waypoints = [waypoints; waypoint];
    else
        disp('Invalid input. Please enter in the format [lon, lat].');
    end
end
waypoints = [waypoints; endPoint];

% Compute ROI buildings
roiBuildings = ComputeROI(buildings, startPoint, endPoint);

% Reduce vertices in ROI buildings
for i = 1:length(roiBuildings)
    roiBuildings(i).vertices = ReduceMapVertices(roiBuildings(i).vertices, nmaxVert, pdwnSmple);
end

% Create 3D geofences for ROI buildings
for i = 1:length(roiBuildings)
    vertices = roiBuildings(i).vertices;
    height = roiBuildings(i).height + safetyBuffer;
    roiBuildings(i).geofence3D = [vertices, repmat(height, size(vertices, 1), 1)];
end

% Construct visibility graph and calculate path
visibilityGraph = constructVisibilityGraph(waypoints, roiBuildings);
[path, totalCost] = dijkstra(visibilityGraph, 1, size(waypoints, 1));

% Generate 3D UAV path
%uavPath3D = generate3DUavPath(waypoints, path, climbAltitude, cruiseAltitude, descendAltitude, roiBuildings);

% Plot the 3D UAV path
%plot3DPath(uavPath3D, buildings, false);

% Handle dangerous zones and re-calculate path
%dangerousZone = createRandomDangerousZone(roiBuildings);
%roiBuildingsWithDanger = [roiBuildings; dangerousZone];
%visibilityGraphUpdated = constructVisibilityGraph(waypoints, roiBuildingsWithDanger);
%[pathUpdated, totalCostUpdated] = dijkstra(visibilityGraphUpdated, 1, size(waypoints, 1));
%optimalUavPathUpdated = generate3DUavPath(waypoints, pathUpdated, climbAltitude, cruiseAltitude, descendAltitude, roiBuildings);

% Plot the updated path avoiding the dangerous zone
%plotMapWithDangerZone(waypoints, optimalUavPathUpdated, buildings, roads, dangerousZone);

% Evaluate different path planning solutions
%[Pturn, Gturn, Dturn] = turnSolution(waypoints, roiBuildings, climbAltitude, cruiseAltitude, descendAltitude, delta_sb);
%[Pconst, Gconst, Dconst] = constantCruiseSolution(waypoints, roiBuildings, cruiseAltitude, descendAltitude,delta_sb);
%[Pterr, Gterr, Dterrain] = terrainFollowerSolution(waypoints, roiBuildings, descendAltitude,delta_sb,osmData);

% Compare costs and select the best solution
%[Cmin, opt] = costCompare(Dturn, Dconst, Dterrain, alpha, beta, gamma);

% Select the optimal path
%%switch opt
    %case 1
       % Ptraj = Pturn;
       % Gtraj = Gturn;
    %case 2
       % Ptraj = Pconst;
      %  Gtraj = Gconst;
   % case 3
   %   Ptraj = Pterr;
 %       Gtraj = Gterr;
%end

% Plot the final optimal path
%plotOptimalPath(waypoints, Ptraj, roiBuildings, roads);

% Plot all possible paths in 3D
%allPaths = findAllPaths(visibilityGraph, 1, size(waypoints, 1));
%uavPaths3D = cellfun(@(p) generate3DUavPath(waypoints, p, climbAltitude, cruiseAltitude, descendAltitude, roiBuildings), allPaths, 'UniformOutput', false);
%plot3DPath(uavPath3D, buildings, true, uavPaths3D);
[pathAStar, totalCostAStar] = aStar(visibilityGraph, 1, size(waypoints, 1), waypoints);
uavPathAStar = generate3DUavPath(waypoints, pathAStar, climbAltitude, cruiseAltitude, descendAltitude, roiBuildings);
plotOptimalPath(waypoints, uavPathAStar, roiBuildings, roads);
function buildings = extractBuildings(osmData)
    buildings = [];
    ways = osmData.osm.way;
    for i = 1:length(ways)
        if isfield(ways{i}, 'tag')
            tags = ways{i}.tag;
            isBuilding = false;
            buildingHeight = 10; % Default height if not specified
            
            if iscell(tags)
                for j = 1:length(tags)
                    if isfield(tags{j}.Attributes, 'k') && strcmp(tags{j}.Attributes.k, 'building')
                        isBuilding = true;
                    end
                    if isfield(tags{j}.Attributes, 'k') && strcmp(tags{j}.Attributes.k, 'height')
                        buildingHeight = str2double(tags{j}.Attributes.v);
                    end
                end
            elseif isstruct(tags)
                if isfield(tags.Attributes, 'k') && strcmp(tags.Attributes.k, 'building')
                    isBuilding = true;
                end
                if isfield(tags.Attributes, 'k') && strcmp(tags.Attributes.k, 'height')
                    buildingHeight = str2double(tags.Attributes.v);
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
                buildings = [buildings; struct('vertices', vertices, 'height', buildingHeight)];
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


function uavPath = generate3DUavPath(waypoints, path, climbAltitude, cruiseAltitude, descendAltitude, roiBuildings)
    uavPath = [];
    for i = 1:length(path)
        waypoint = waypoints(path(i), :);
        if i == 1  % Start: Climb
            altitude = climbAltitude;
        elseif i == length(path)  % End: Descend
            altitude = descendAltitude;  % Assumes ground level at the destination
        else
            % Adjust altitude based on proximity to geofences
            altitude = cruiseAltitude;
            for j = 1:length(roiBuildings)
                if inpolygon(waypoint(1), waypoint(2), roiBuildings(j).vertices(:,1), roiBuildings(j).vertices(:,2))
                    altitude = max(altitude, roiBuildings(j).height + 5); % Maintain a safety buffer
                end
            end
        end
        uavPath = [uavPath; waypoint, altitude];
    end
end

function plot3DPath(uavPath, buildings, showAllPaths, allPaths)
    figure;
    hold on;

    % Plot buildings as 3D obstacles
    for i = 1:length(buildings)
        vertices = buildings(i).vertices;
        height = buildings(i).height;
        fill3(vertices(:,1), vertices(:,2), height * ones(size(vertices, 1), 1), 'r', 'FaceAlpha', 0.5);
        fill3(vertices(:,1), vertices(:,2), zeros(size(vertices, 1), 1), 'r', 'FaceAlpha', 0.5);
        for j = 1:size(vertices, 1)
            plot3([vertices(j,1), vertices(j,1)], [vertices(j,2), vertices(j,2)], [0, height], 'k');
        end
    end

    if showAllPaths
        % Plot all possible paths
        colors = lines(length(allPaths));
        for i = 1:length(allPaths)
            currentPath = allPaths{i};
            for j = 1:size(currentPath, 1)-1
                p1 = currentPath(j, :);
                p2 = currentPath(j+1, :);
                plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'Color', colors(i,:), 'LineWidth', 1);
            end
        end
    end

    % Plot the UAV path with altitude changes
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
        
        plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'Color', lineColor, 'LineWidth', 2);
    end
    
    xlabel('Longitude');
    ylabel('Latitude');
    zlabel('Altitude');
    title('3D UAV Path with Obstacles');
    grid on;
    view(3);
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
    
    dangerousZone = struct('vertices', vertices, 'height', 10); % Arbitrary height
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
    
    title('Map with Dangerous Zone and UAV Path');
    xlabel('Longitude');
    ylabel('Latitude');
    grid on;
    hold off;
end

% Function to check if a line segment intersects with any obstacles
function intersects = checkIntersection(p1, p2, obstacles)
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

% Function to check if two line segments intersect
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

% Helper function to determine the direction of the triplet (p1, p2, p3)
function d = direction(p1, p2, p3)
    d = (p3(1) - p1(1)) * (p2(2) - p1(2)) - (p2(1) - p1(1)) * (p3(2) - p1(2));
end

% Helper function to check if point p lies on the line segment [p1, p2]
function onSeg = onSegment(p1, p2, p)
    onSeg = min(p1(1), p2(1)) <= p(1) && p(1) <= max(p1(1), p2(1)) && ...
            min(p1(2), p2(2)) <= p(2) && p(2) <= max(p1(2), p2(2));
end

function [Pturn, Gturn, Dturn] = turnSolution(waypoints, roiBuildings, climbAltitude, cruiseAltitude, descendAltitude, delta_sb)
    % Use the existing visibility graph and Dijkstra's algorithm
    visibilityGraph = constructVisibilityGraph(waypoints, roiBuildings);
    [path, ~] = dijkstra(visibilityGraph, 1, size(waypoints, 1));
    
    % Generate UAV path
    Pturn = generate3DUavPath(waypoints, path, climbAltitude, cruiseAltitude, descendAltitude, roiBuildings);
    
    % Wrap the trajectory with 3D geofence volumes
    Gturn = createGeofenceVolumes(Pturn, roiBuildings, delta_sb);
    
    % Calculate distance
    Dturn = getPathDistance(Pturn);
end
function [Pconst, Gconst, Dconst] = constantCruiseSolution(waypoints, roiBuildings, cruiseAltitude, descendAltitude, delta_sb)
    % Determine the cruise altitude needed to avoid all obstacles
    maxBuildingHeight = max([roiBuildings.height]) + delta_sb;
    cruiseAltitude = max(cruiseAltitude, maxBuildingHeight);

    % Generate UAV path: straight line at cruise altitude
    Pconst = [
        waypoints(1, :), cruiseAltitude;
        waypoints(end, :), cruiseAltitude;
        waypoints(end, :), descendAltitude;
    ];

    % Wrap the trajectory with 3D geofence volumes
    Gconst = createGeofenceVolumes(Pconst, roiBuildings, delta_sb);

    % Calculate distance
    Dconst = getPathDistance(Pconst);
end

function [Pterr, Gterr, Dterrain] = terrainFollowerSolution(waypoints, roiBuildings, descendAltitude, delta_sb,osmData)
    % Initialize the path
    Pterr = [];
    for i = 1:size(waypoints, 1)
        waypoint = waypoints(i, :);

        % Determine the minimum safe altitude (terrain + safety buffer)
        terrainAltitude = getTerrainAltitude(waypoint,osmData); % Stub function to get terrain altitude
        maxObstacleHeight = 0;
        for j = 1:length(roiBuildings)
            if inpolygon(waypoint(1), waypoint(2), roiBuildings(j).vertices(:,1), roiBuildings(j).vertices(:,2))
                maxObstacleHeight = max(maxObstacleHeight, roiBuildings(j).height + delta_sb);
            end
        end

        safeAltitude = max(terrainAltitude + delta_sb, maxObstacleHeight);
        Pterr = [Pterr; waypoint, safeAltitude];
    end

    % Add descent to destination
    Pterr = [Pterr; waypoints(end, :), descendAltitude];

    % Wrap the trajectory with 3D geofence volumes
    Gterr = createGeofenceVolumes(Pterr, roiBuildings, delta_sb);

    % Calculate distance
    Dterrain = getPathDistance(Pterr);
end

function [Cmin, opt] = costCompare(Dturn, Dconst, Dterrain, alpha, beta, gamma)
    % Define costs for each path planning solution
    % Note: For simplicity, we consider only distance in this example.
    % You could also include power consumption and time delay based on your specific needs.

    % Example costs: simply using distance as the cost factor
    Cturn = alpha * Dturn;     % Turn solution cost
    Cconst = alpha * Dconst;   % Constant cruise altitude cost
    Cterrain = alpha * Dterrain; % Terrain following cost

    % Compare the costs and choose the minimum
    [Cmin, opt] = min([Cturn, Cconst, Cterrain]);
end
function geofences = createGeofenceVolumes(path, roiBuildings, delta_sb)
    % Initialize the geofences structure
    geofences = [];

    % Loop through each segment of the path
    for i = 1:size(path, 1)-1
        % Get the start and end points of the segment
        p1 = path(i, 1:2);
        p2 = path(i+1, 1:2);

        % Loop through each building in the region of interest (ROI)
        for j = 1:length(roiBuildings)
            vertices = roiBuildings(j).vertices;
            height = roiBuildings(j).height + delta_sb; % Add the safety buffer to the height

            % Check if the path segment intersects the building's area
            if isLineIntersectingPolygon(p1, p2, vertices)
                % Create a 3D geofence volume for this segment and building
                geofence.vertices = vertices;
                geofence.height = height;
                geofences = [geofences; geofence];
            end
        end
    end
end

function intersects = isLineIntersectingPolygon(p1, p2, polygon)
    % Initialize the intersection flag
    intersects = false;

    % Number of vertices in the polygon
    numVertices = size(polygon, 1);

    % Loop through each edge of the polygon
    for i = 1:numVertices
        % Get the current edge of the polygon
        p3 = polygon(i, :);
        p4 = polygon(mod(i, numVertices) + 1, :);

        % Check if the line segment intersects the polygon edge
        if segmentsIntersect(p1, p2, p3, p4)
            intersects = true;
            return;
        end
    end
end
function distance = getPathDistance(path)
    % Initialize the distance
    distance = 0;

    % Loop through each segment of the path
    for i = 1:size(path, 1) - 1
        % Calculate the Euclidean distance between consecutive points
        segmentDistance = sqrt(sum((path(i+1, :) - path(i, :)).^2));
        
        % Add the segment distance to the total distance
        distance = distance + segmentDistance;
    end
end

function altitude = getTerrainAltitude(waypoint, osmData)
    % Extract nodes with elevation data
    nodes = osmData.osm.node;
    elevations = [];
    locations = [];
    
    for i = 1:length(nodes)
        if isfield(nodes{i}, 'tag')
            tags = nodes{i}.tag;
            
            % Check if tags is a struct or cell array
            if isstruct(tags)
                tags = {tags};  % Convert struct to cell array for uniform processing
            end
            
            for j = 1:length(tags)
                if strcmp(tags{j}.Attributes.k, 'ele')
                    ele = str2double(tags{j}.Attributes.v);
                    lat = str2double(nodes{i}.Attributes.lat);
                    lon = str2double(nodes{i}.Attributes.lon);
                    elevations = [elevations; ele];
                    locations = [locations; lon, lat];
                end
            end
        end
    end
    
    % If no elevation data, return a default altitude
    if isempty(elevations)
        altitude = 0;  % Default to sea level
        return;
    end
    
    % Use scatteredInterpolant for interpolation over scattered data
    F = scatteredInterpolant(locations(:,1), locations(:,2), elevations, 'linear', 'none');
    altitude = F(waypoint(1), waypoint(2));
    
    % Handle cases where the interpolation results in NaN (out of bounds)
    if isnan(altitude)
        altitude = 0;  % Default to sea level if out of bounds
    end
end

function plotOptimalPath(waypoints, Ptraj, roiBuildings, roads)
    figure;
    hold on;
    
    % Plot buildings
    for i = 1:length(roiBuildings)
        fill(roiBuildings(i).vertices(:,1), roiBuildings(i).vertices(:,2), 'r', 'FaceAlpha', 0.5);
    end
    
    % Plot roads
    for i = 1:length(roads)
        plot(roads{i}(:,1), roads{i}(:,2), 'g--');
    end
    
    % Plot waypoints
    plot(waypoints(:,1), waypoints(:,2), 'bo-', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    
    % Plot optimal path
    plot(Ptraj(:,1), Ptraj(:,2), 'r-', 'LineWidth', 2);
    
    % Annotate waypoints and path segments
    for i = 1:size(waypoints, 1)
        text(waypoints(i, 1), waypoints(i, 2), sprintf('Waypoint %d', i), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    end
    
    xlabel('Longitude');
    ylabel('Latitude');
    title('Optimal Path with Waypoints, Buildings, and Roads');
    grid on;
    hold off;
end

function allPaths = findAllPaths(visibilityGraph, startNode, endNode)
    allPaths = {};
    stack = {startNode};   % Stack to keep track of nodes to visit
    pathStack = {startNode};  % Stack to keep track of the current path
    
    while ~isempty(stack)
        currentNode = stack{end};  % Get the last node added to the stack
        currentPath = pathStack{end};  % Get the path to the last node
        stack(end) = [];  % Pop the node from the stack
        pathStack(end) = [];  % Pop the path from the stack
        
        if currentNode == endNode
            allPaths = [allPaths; {currentPath}];  % Add the path to the list of all paths
        else
            neighbors = find(visibilityGraph(currentNode, :) < inf);  % Find all neighbors
            for i = 1:length(neighbors)
                neighbor = neighbors(i);
                if ~ismember(neighbor, currentPath)
                    stack = [stack; neighbor];  % Push the neighbor to the stack
                    pathStack = [pathStack; [currentPath, neighbor]];  % Push the new path to the stack
                end
            end
        end
    end
end
function plot3DPaths(waypoints, uavPaths, buildings)
    figure;
    hold on;
    
    % Plot buildings as 3D obstacles
    for i = 1:length(buildings)
        vertices = buildings(i).vertices;
        height = buildings(i).height;
        fill3(vertices(:,1), vertices(:,2), height * ones(size(vertices, 1), 1), 'r', 'FaceAlpha', 0.5);
        fill3(vertices(:,1), vertices(:,2), zeros(size(vertices, 1), 1), 'r', 'FaceAlpha', 0.5);
        for j = 1:size(vertices, 1)
            plot3([vertices(j,1), vertices(j,1)], [vertices(j,2), vertices(j,2)], [0, height], 'k');
        end
    end
    
    % Plot all UAV paths
    colors = lines(length(uavPaths));
    for i = 1:length(uavPaths)
        path = uavPaths{i};
        plot3(path(:,1), path(:,2), path(:,3), 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    
    % Plot waypoints
    plot3(waypoints(:,1), waypoints(:,2), zeros(size(waypoints, 1), 1), 'bo-', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    
    xlabel('Longitude');
    ylabel('Latitude');
    zlabel('Altitude');
    title('3D Paths with Waypoints and Buildings');
    grid on;
    view(3);
    hold off;
end


function [path, totalCost] = aStar(visibilityGraph, startIdx, endIdx, waypoints)
    n = size(visibilityGraph, 1);
    openSet = startIdx;
    closedSet = [];
    gScore = inf(1, n);
    gScore(startIdx) = 0;
    fScore = inf(1, n);
    fScore(startIdx) = heuristicCostEstimate(waypoints(startIdx, :), waypoints(endIdx, :));

    cameFrom = nan(1, n);

    while ~isempty(openSet)
        % Get node with the lowest fScore
        [~, idx] = min(fScore(openSet));
        current = openSet(idx);

        if current == endIdx
            [path, totalCost] = reconstructPath(cameFrom, current);
            return;
        end

        openSet(idx) = [];
        closedSet = [closedSet, current];

        neighbors = find(visibilityGraph(current, :) < inf);
        for neighbor = neighbors
            if ismember(neighbor, closedSet)
                continue;
            end

            tentative_gScore = gScore(current) + visibilityGraph(current, neighbor);

            if ~ismember(neighbor, openSet)
                openSet = [openSet, neighbor];
            elseif tentative_gScore >= gScore(neighbor)
                continue;
            end

            % Update the path
            cameFrom(neighbor) = current;
            gScore(neighbor) = tentative_gScore;
            fScore(neighbor) = gScore(neighbor) + heuristicCostEstimate(waypoints(neighbor, :), waypoints(endIdx, :));
        end
    end

    error('A*: No path found');
end

function h = heuristicCostEstimate(point1, point2)
    h = norm(point1 - point2); % Euclidean distance
end

function [path, totalCost] = reconstructPath(cameFrom, current)
    path = current;
    while ~isnan(cameFrom(current))
        current = cameFrom(current);
        path = [current, path];
    end
    totalCost = 0; % Placeholder, calculate if needed
end