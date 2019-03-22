profile on

% Set given variables
syms V T L1 L2 L3

% Find chorde parameter
l1 = V*T;
l2 = V*T;
lf = l1 + l2;

d13 = L1 - L3;

alpha13 = atan(d13/lf);
chorde13 = abs(lf/cos(alpha13));

% Define support variables
x = 1;
y = 2;

p1 = [L1 0];
p2 = [L2 V*T];
p3 = [L3 2*V*T];

% Find height of sector and radii
Ac = p1(y) - p3(y);
Bc = p3(x) - p1(x);
Cc = p1(x)*p3(y) - p3(x)*p1(y);

h = abs(Ac*p2(x) + Bc*p2(y) +Cc)/sqrt(Ac^2 + Bc^2);
R = (h^2 + (chorde13/2)^2)/(2*h);

% Find center coordinates
q = chorde13; % sqrt((p1(x) - p2(x))^2 + (p1(y) - p2(y))^2);
pqc = [(p1(x)+p3(x))/2 (p1(y)+p3(y))/2];

cx = pqc(x) - sqrt(R^2-(q/2)^2)*(p1(y)-p3(y))/q;
cy = pqc(y) - sqrt(R^2-(q/2)^2)*(p3(x)-p1(x))/q;

profile off
profile viewer
% Testing
vars_set = {V, T, L1, L2, L3};
vals_set = {1000, 1, 1600, 1700, 2500};

sol_chorde = subs(chorde13, vars_set, vals_set);
sol_R = subs(R, vars_set, vals_set);
sol_h = subs(h, vars_set, vals_set);
sol_cx = subs(cx, vars_set, vals_set);
sol_cy = subs(cy, vars_set, vals_set);

pnt1 = subs(p1, vars_set, vals_set);
pnt2 = subs(p2, vars_set, vals_set);
pnt3 = subs(p3, vars_set, vals_set);

car_p1 = [0 0];
car_p2 = [0 V*T];
car_p3 = [0 2*V*T];

car_pnt1 = subs(car_p1, vars_set, vals_set);
car_pnt2 = subs(car_p2, vars_set, vals_set);
car_pnt3 = subs(car_p3, vars_set, vals_set);

square_pnts_x = [pnt1(x); pnt2(x); pnt3(x)];
square_pnts_y = [pnt1(y); pnt2(y); pnt3(y)];

car_pnts_x = [car_pnt1(x), car_pnt2(x), car_pnt3(x)];
car_pnts_y = [car_pnt1(y), car_pnt2(y), car_pnt3(y)];

sq_x = sol_cx;
sq_y = sol_cy;

% Render circle
th = 0:pi/50:2*pi;
xcircle = sol_R*cos(th) + sq_x;
ycircle = sol_R*sin(th) + sq_y;

% Last one - distance from third to UV
dist = sqrt((sq_x-car_pnt3(x))^2 + (sq_y-car_pnt3(y))^2);
dist = double(dist);

disp("Radii:");
disp(double(sol_R));

disp("Center:");
disp(double([sq_x sq_y]));

disp("Distance:");
disp(dist);

plot(square_pnts_x, square_pnts_y, '*'); hold;
plot(car_pnts_x, car_pnts_y, '*r');
plot(sq_x, sq_y, '*m'); 
plot(xcircle, ycircle); 
hold off;
