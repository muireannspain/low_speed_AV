% uiopen('EllipseWaypoints.csv');
EllipseWaypoints1.Properties.VariableNames{1} = 'X';
EllipseWaypoints1.Properties.VariableNames{2} = 'Y';
x = EllipseWaypoints1.X;
y = EllipseWaypoints1.Y;

for i = 1:600
    plot(x(i),y(i),'*','color','r')
    hold on
    pause(0.3)
end