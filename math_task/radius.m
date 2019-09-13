syms k R beta h

a = k/2;
b = R-h;

R = sqrt(a^2+b^2);

solve(sqrt(a^2+b^2) == R, R)
