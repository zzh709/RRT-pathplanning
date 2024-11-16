clc;
clear;
%% Create and plot obstacles
xrange = [0 1200];
yrange = [590 1200];
zrange = [0 1200];
 
% Create grid points
[X, Y] = meshgrid(xrange(1):15:xrange(2), yrange(1):15:yrange(2));
 
% Calculate the height of the grid points
R = sqrt(((X-600)/1.5).^2 + ((Y-600)/1.5).^2);
Z = 90 * sin(R/800) .* exp(-R/800) + 600*exp(-((X-600).^2 + (Y-600).^2)/300000);
 
% Limit the height within the coordinate system range
Z(Z > zrange(2)) = zrange(2);
Z(Z < zrange(1)) = zrange(1);
 
% Convert grid points to scatter points
scatter3(X(:),Y(:), Z(:), 'filled','MarkerFaceAlpha',.2,'MarkerEdgeAlpha',.2);
 
map = [0.99 0.90 0.79
    0.99 0.90 0.79
    0.99 0.90 0.79
    0.99 0.90 0.79
    0.99 0.90 0.79
    0.99 0.90 0.79];
% Plot 3D surface
surf(X, Y, Z,'EdgeColor','0.5 0.54 0.53');
colormap(map);
axis equal;
 
%% Create start and goal positions and plot them
aa = 600;
start = [0 600 300];
goal = [1000 600 1200];
hold on;
scatter3(start(1),start(2),start(3),'filled','g');
scatter3(goal(1),goal(2),goal(3),'filled','g');
 
%% Annotate the plot
text(start(1)-40,start(2),start(3)+50,'start','color','g');
text(goal(1),goal(2),goal(3),'goal4','color','g');
view(3);
grid on;
axis equal;
axis([0 1200 0 1200 0 1200]);
xlabel('x');
ylabel('y');
zlabel('z');
 
%% RRT method to generate obstacle-avoiding trajectory
path = RRT(start,goal,Z);
 
function path = RRT(start,goal,Z)
%% Define RRT parameters
stepSize = 15;                           
maxIterTimes = 10000;                     
iterTime = 0;                            
threshold = 20;                          
RRTree = double([start -1]);             
 
p = 0.3;
vlei = 0;
V=0;
c=0;
leng = 0;
searchSize = [1000,1000,1000];
calcDis = @(a,b) sqrt((b(:,1)-a(1,1)).^2 + (b(:,2)-a(1,2)).^2 + (b(:,3)-a(1,3)).^2);
goald = 0;
 
%% Find RRT path
tic                       
pathFound = false;
while iterTime <= maxIterTimes
    iterTime = iterTime +1;
    disp(iterTime);
    if rand < 0.999
        sample = [rand() *1000, 600,rand() *1200];  
    else
        sample = [1000 600 1200];
    end
    [val,nearIndex] = min(calcDis(sample, RRTree(:,1:3)),[],1);       
    closestNode = RRTree(nearIndex,1:3);
    leng=length(RRTree);
    growVec = sample - closestNode;
    growVec = growVec/sqrt(sum(growVec.^2));
    newPoint = closestNode + growVec*stepSize;
    
    feasible = collisionDetec(newPoint,closestNode);   
    
    if V<= 3000                        
        if (~feasible)&&(dis(newPoint,[600,600,0])>dis(closestNode,[600,600,0]))   
            continue;
        end
    end
    
    RRTree = [RRTree;newPoint nearIndex];
    plot3([closestNode(1) newPoint(1)],[closestNode(2) newPoint(2)],[closestNode(3) newPoint(3)],'LineWidth',1,'Color',[0.25,0.41,1]);
    pause(0.01);
    if newPoint(3) <= ZXY(round(newPoint(1)),round(newPoint(1)))   
        V = V + abs((newPoint(1)-closestNode(1))*2);
    end
    goald = newPoint(3);
    
    disp(newPoint)
    
    if ((newPoint(3)<ZXY(newPoint(1),newPoint(2)))&&((newPoint(3)+20)>ZXY(newPoint(1),newPoint(2))))
        V2 = 2800+0.5*(1000-newPoint(1))*500;
        if V<V2
           c = -1;
        end
    end   
     
    p = 0.1+V/3000+c;   
    c=0;
    
    if sqrt(sum((newPoint - goal).^2)) <= threshold
        pathFound = true;
        break;           
    end 
end
 
if ~pathFound
    disp('no path found. maximum attempts reached');
end
 
toc
 
%% Trace back the path
path = goal;
lastNode = nearIndex;           
while lastNode >= 0
    path =[RRTree(lastNode,1:3); path];         
    lastNode = RRTree(lastNode, 4);             
end
plot3(path(:,1), path(:,2), path(:,3),'LineWidth',1,'Color','r');
disp(path)
 
end
 
function feasible = collisionDetec(newPoint,closestNode)
feasible = true;
checkVec = newPoint - closestNode;    
for i = 0:0.5:sqrt(sum(checkVec.^2))
    checkPoint = closestNode + i.*(checkVec/sqrt(sum(checkVec.^2)));     
    a = round(checkPoint(1));
    b = round(checkPoint(2));
    if checkPoint(3) >= ZXY(a, b)
        feasible = false;
    break; 
    end
end
end
 
function ZZ = ZXY(x, y)
r = sqrt(((x-600)/1.5).^2 + ((y-600)/1.5).^2);
ZZ = 90 * sin(r/800) .* exp(-r/800) + 600*exp(-((x-600).^2 + (y-600).^2)/300000);
end
 
function distance = dis(X,Y)
distance = norm(X-Y);
end