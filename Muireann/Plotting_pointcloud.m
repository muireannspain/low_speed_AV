ptCloud = pcread('finalCloud.pcd');
roi=[-10.5065451,65.1853027,-1,0.5,-31.5815582,1.0809262e+02];
indices = findPointsInROI(ptCloud,roi);
ptCloudB = select(ptCloud,indices);
[ptCloudOut,ind] = removeInvalidPoints(ptCloudB);
 pcshow(ptCloudOut);

locations = ptCloudOut.Location;

figure()
hold on
grid on
%plot obstacles
plot(locations(:,1), locations(:,3),  '.')


% %calculate waypoints

%case 1, plot as an ellipse
x1=8;
x2=13;
y1=-3;
y2=101;
e=0.97;
a = 1/2*sqrt((x2-x1)^2+(y2-y1)^2);
b = a*sqrt(1-e^2);
t = linspace(0,200,100);
X = a*cos(t);
Y = b*sin(t);
w = atan2(y2-y1,x2-x1);
x = (x1+x2)/2 + X*cos(w) - Y*sin(w);
y = (y1+y2)/2 + X*sin(w) + Y*cos(w);
plot(x,y,'r.')
axis equal

% %case 2, plot a straight line
% x=linspace(-1,4,40);
% y=linspace(5,98,40);
% plot(x,y,'r.')
% axis equal

% %case 3, "parking maneuver"
% x = [-1 -0.5 5 0 1 6 1.3];
% y = [5 10.5 17 22 39 42 48];
% plot(x,y,'r.')
% axis equal