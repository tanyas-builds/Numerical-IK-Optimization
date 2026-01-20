% Analytical solution to Inverse Kinematic
a1 = 1; 
a2 = 2; 

% The forward kinematic

F = @(th1,th2)[a1*cos(th1)+a2*cos(th1 + th2);
               a1*sin(th1)+a2*sin(th1 + th2)];
C = [1 1]; 
f = @(u)F(u(1),u(2)) - C;
u0 = [1;1];
u = fsolve(f,u0);
f(u)


