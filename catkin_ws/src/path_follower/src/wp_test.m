uiopen('EllipseWaypoints.csv')
EllipseWaypoints.Properties.VariableNames{1} = 'X';
EllipseWaypoints.Properties.VariableNames{2} = 'Y';
x = EllipseWaypoints.X;
y = EllipseWaypoints.Y;

for i = 1:600
    plot(x(i),y(i),'*','color','r')
    hold on
    pause(0.3)
end