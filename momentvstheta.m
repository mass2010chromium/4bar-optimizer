AOA;
cpDistance;
pivotX;
pivotY;
lift;
drag;
Ftotal;
length = 5;

slope = -lift/drag
cpX = cpDistance*cosd(AOA);
cpY = cpDistance*sind(AOA);
b = cpY-slope*cpX % y=mx+b
pivotHypotenuse = sqrt(pivotX^2+pivotY^2);
pivotAngle = atand(pivotY/pivotX);
x = pivotHypotenuse*cosd(AOA-pivotAngle);
y = pivotHypotenuse*sind(AOA-pivotAngle);
momentArm = abs(-slope*x+y-b)/sqrt(slope^2+1); % dot product
moment = Ftotal*momentArm