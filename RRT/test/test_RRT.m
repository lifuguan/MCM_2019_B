%% RRT parameters
map=im2bw(imread('test_map.jpg')); % input map read from a bmp file. for new maps write the file name here
source=[10 10]; % source position in Y, X format
goal=[490 490]; % goal position in Y, X format
stepsize = 20;  % size of each step of the RRT
threshold = 20; % nodes closer than this threshold are taken as almost the same
maxFailedAttempts = 10000;
display = true; % display of RRT

if ~feasiblePoint(source,map), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end
if display,imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k'); end

tic;  % tic-toc: Functions for Elapsed Time

RRTree = double([source -1]); % RRT rooted at the source, representation node and parent index
failedAttempts = 0;
counter = 0;
pathFound = false;

while failedAttempts <= maxFailedAttempts  % loop to grow RRTs
    %% chooses a random configuration
    if rand < 0.5
        sample = rand(1,2) .* size(map);   % random sample
    else
        sample = goal; % sample taken as goal to bias tree generation to goal
    end
	
    %% selects the node in the RRT tree that is closest to qrand
    [A, I] = min( distanceCost(RRTree(:,1:2),sample) ,[],1); % find the minimum value of each column
    closestNode = RRTree(I(1),1:2);
	
    %% moving from qnearest an incremental distance in the direction of qrand
    theta = atan2(sample(1)-closestNode(1),sample(2)-closestNode(2));  % direction to extend sample to produce new node
    newPoint = double(int32(closestNode(1:2) + stepsize * [sin(theta)  cos(theta)]));
    if ~checkPath(closestNode(1:2), newPoint, map) % if extension of closest node in tree to the new point is feasible
        failedAttempts = failedAttempts + 1;
        continue;
    end
	
    if distanceCost(newPoint,goal) < threshold, pathFound = true; break; end % goal reached
	
    [A, I2] = min( distanceCost(RRTree(:,1:2),newPoint) ,[],1); % check if new node is not already pre-existing in the tree
    if distanceCost(newPoint,RRTree(I2(1),1:2)) < threshold, failedAttempts = failedAttempts + 1; continue; end 
	
    RRTree = [RRTree; newPoint I(1)]; % add node
    failedAttempts = 0;
    if display, line([closestNode(2);newPoint(2)],[closestNode(1);newPoint(1)]);counter = counter + 1; M(counter) = getframe; end % Capture movie frame 
end

% getframe returns a movie frame, which is a structure having two fields
if display && pathFound, line([closestNode(2);goal(2)],[closestNode(1);goal(1)]); counter = counter+1;M(counter) = getframe; end

if display, disp('click/press any key'); waitforbuttonpress; end
if ~pathFound, error('no path found. maximum attempts reached'); end

%% retrieve path from parent information
path = [goal];
prev = I(1);
while prev > 0
    path = [RRTree(prev,1:2); path];
    prev = RRTree(prev,3);
end

pathLength = 0;
for i=1:length(path)-1, pathLength = pathLength + distanceCost(path(i,1:2),path(i+1,1:2)); end % calculate path length
fprintf('processing time=%d \nPath Length=%d \n\n', toc, pathLength); 
imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k');
line(path(:,2),path(:,1));


%% distanceCost.m
function h=distanceCost(a,b)
	h = sqrt(sum((a-b).^2, 2));
end
	
%% checkPath.m	
function feasible=checkPath(n,newPos,map)
feasible=true;
dir=atan2(newPos(1)-n(1),newPos(2)-n(2));
for r=0:0.5:sqrt(sum((n-newPos).^2))
    posCheck=n+r.*[sin(dir) cos(dir)];
    if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map) && ... 
            feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
        feasible=false;break;
    end
    if ~feasiblePoint(newPos,map), feasible=false; end
end
end


%% feasiblePoint.m
function feasible=feasiblePoint(point,map)
feasible=true;
% check if collission-free spot and inside maps
if ~(point(1)>=1 && point(1)<=size(map,1) && point(2)>=1 && point(2)<=size(map,2) && map(point(1),point(2))==1)
    feasible=false;
end
end