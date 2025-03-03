% Load OSM data
filename = 'map.osm';  
osmData = xml2struct(filename);
gifFilename = 'uav_path_planning.gif';
delayTime = 0.5; 

% Extract detailed OSM data
buildings = extractBuildings(osmData);
roads = extractRoads(osmData);

% Define waypoints, start, and end points
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

% Set up the initial map
figure;
hold on;
title('UAV Path in Reinforcement Learning with Obstacle Avoidance');
xlabel('Longitude');
ylabel('Latitude');
grid on;

% Plot buildings (geofences)
for i = 1:length(buildings)
    fill3(buildings(i).vertices(:,1), buildings(i).vertices(:,2), buildings(i).height * ones(size(buildings(i).vertices, 1), 1), 'r', 'FaceAlpha', 0.5);
end

% Plot roads
for i = 1:length(roads)
    plot3(roads{i}(:,1), roads{i}(:,2), zeros(size(roads{i}, 1), 1), 'g--');
end

% Plot waypoints
plot3(waypoints(:,1), waypoints(:,2), zeros(size(waypoints, 1), 1), 'bo-', 'MarkerSize', 8, 'MarkerFaceColor', 'b');

% Parameters for reinforcement learning
numEpisodes = 50; % Number of episodes
maxSteps = 50; % Max steps per episode
learningRate = 0.1; % Learning rate for Q-learning
discountFactor = 0.9; % Discount factor for future rewards
epsilon = 0.1; % Exploration factor
penalty = -1000; % Penalty for entering a geofence
threshold = 0.001; % Distance threshold to consider the goal reached

% Initialize Q-values (state-action pairs)
Q = zeros(size(waypoints, 1), size(waypoints, 1));

% Reinforcement Learning Loop
for episode = 1:numEpisodes
    % Initialize UAV position and path
    currentPosIdx = 1; % Start at the first waypoint (start point)
    uavPath = waypoints(currentPosIdx, :);
    
    % Loop through steps within each episode
    for step = 1:maxSteps
        % Select action (next waypoint) using epsilon-greedy policy
        if rand < epsilon
            % Explore: select a random action (next waypoint)
            nextPosIdx = randi(size(waypoints, 1));
        else
            % Exploit: select the best action based on Q-values
            [~, nextPosIdx] = max(Q(currentPosIdx, :));
        end
        
        % Get the position coordinates of the next waypoint
        nextPos = waypoints(nextPosIdx, :);
        
        % Check if the path to the next waypoint intersects with any geofence
        if checkIntersection3D(uavPath(end, :), [nextPos, 0], buildings)
            reward = penalty; % Apply penalty if it intersects a geofence
        else
            % Calculate the reward (negative distance to the goal)
            reward = -norm(nextPos - endPoint);
        end
        
        % Update Q-value using the Q-learning formula
        Q(currentPosIdx, nextPosIdx) = Q(currentPosIdx, nextPosIdx) + ...
            learningRate * (reward + discountFactor * max(Q(nextPosIdx, :)) - Q(currentPosIdx, nextPosIdx));
        
        % Update the UAV path
        uavPath = [uavPath; nextPos];
        
        % Plot the updated UAV path
        plot3(uavPath(:,1), uavPath(:,2), zeros(size(uavPath, 1), 1), 'b-', 'LineWidth', 2);
        plot3(waypoints(currentPosIdx, 1), waypoints(currentPosIdx, 2), 0, 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r'); % Current position
        plot3(nextPos(1), nextPos(2), 0, 'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g'); % Next position
        
        % Force the plot to update
        drawnow;
        
        % Pause to make the steps visible
        pause(0.020);
        
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);

        % Write to the GIF file
        if episode == 1 && step == 1
            imwrite(imind, cm, gifFilename, 'gif', 'Loopcount', inf, 'DelayTime', delayTime);
        else
            imwrite(imind, cm, gifFilename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
        end
        
        % Update the current position index
        currentPosIdx = nextPosIdx;
        
        % Check if the UAV has reached the destination
        if norm(nextPos - endPoint) < threshold
            disp(['Reached the goal in episode ', num2str(episode), ' at step ', num2str(step)]);
            break;
        end
    end
    
    % Introduce danger zone after 35 episodes
    if episode == 35
        disp('Introducing danger zone after episode 35.');
        dangerousZone = createDangerZone([-73.976234, 40.765643], 0.001); % Specify the center and size of the danger zone
        buildings = [buildings; dangerousZone]; % Add the danger zone to the buildings list
    end
    
    % After each episode, reset the plot (optional)
    clf;
    hold on;
    title('UAV Path in Reinforcement Learning with Obstacle Avoidance');
    xlabel('Longitude');
    ylabel('Latitude');
    grid on;
    
    % Re-plot buildings (geofences)
    for i = 1:length(buildings)
        fill3(buildings(i).vertices(:,1), buildings(i).vertices(:,2), buildings(i).height * ones(size(buildings(i).vertices, 1), 1), 'r', 'FaceAlpha', 0.5);
    end
    
    % Re-plot roads
    for i = 1:length(roads)
        plot3(roads{i}(:,1), roads{i}(:,2), zeros(size(roads{i}, 1), 1), 'g--');
    end
    
    % Re-plot waypoints
    plot3(waypoints(:,1), waypoints(:,2), zeros(size(waypoints, 1), 1), 'bo-', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
end

% Plot the final optimal path found using Q-values
optimalPath = waypoints(1, :);
currentPosIdx = 1; % Start at the first waypoint (start point)
while currentPosIdx ~= size(waypoints, 1)
    [~, nextPosIdx] = max(Q(currentPosIdx, :));
    optimalPath = [optimalPath; waypoints(nextPosIdx, :)];
    currentPosIdx = nextPosIdx;
end
plot3(optimalPath(:,1), optimalPath(:,2), zeros(size(optimalPath, 1), 1), 'k-', 'LineWidth', 2);
title('Final Optimal Path After Danger Zone Introduction');
hold off;

% Function to plot the optimal path
function plotOptimalPath(Q, waypoints, titleText)
    figure;
    hold on;
    optimalPath = waypoints(1, :);
    currentPosIdx = 1;
    while currentPosIdx ~= size(waypoints, 1)
        [~, nextPosIdx] = max(Q(currentPosIdx, :));
        optimalPath = [optimalPath; waypoints(nextPosIdx, :)];
        currentPosIdx = nextPosIdx;
    end
    plot3(optimalPath(:,1), optimalPath(:,2), zeros(size(optimalPath, 1), 1), 'k-', 'LineWidth', 2);
    title(['Optimal Path ', titleText]);
    xlabel('Longitude');
    ylabel('Latitude');
    grid on;
    hold off;
end

% Function to create a danger zone (a square polygon)
function dangerousZone = createDangerZone(center, size)
    halfSize = size / 2;
    vertices = [
        center(1) - halfSize, center(2) - halfSize;
        center(1) + halfSize, center(2) - halfSize;
        center(1) + halfSize, center(2) + halfSize;
        center(1) - halfSize, center(2) + halfSize;
    ];
    dangerousZone.vertices = vertices;
    dangerousZone.height = 15; % Arbitrary height for the danger zone
end

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

function intersects = checkIntersection3D(p1, p2, buildings)
    % IntersecTion with 3d geofence check 
    intersects = false;
    for i = 1:length(buildings)
        vertices = buildings(i).vertices;
        height = buildings(i).height;
        for j = 1:size(vertices, 1)
            p3 = vertices(j, :);
            p4 = vertices(mod(j, size(vertices, 1)) + 1, :);
            if segmentsIntersect(p1, p2, [p3, 0], [p4, height])
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