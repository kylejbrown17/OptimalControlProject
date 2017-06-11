%% Minimum Energy Example

toms t
t_f = 1;
V = 2.153;
speed = 2.153; xopt = 0; yopt = 0;  vxopt = 0; vyopt = 0; thetaopt = 0;

for n=[4 15 30]
    % Create a new phase and states, using n collocation points
    p = tomPhase('p', t, 0, t_f, n);
    setPhase(p);
    tomStates x y vx vy
    tomControls theta
    % Interpolate an initial guess for the n collocation points
    x0 = {V == speed
        icollocate({x == xopt; y == yopt; vx == vxopt; vy == vyopt})
        collocate(theta == thetaopt)};
    % Box constraints
    cbox = {0 <= V <= 100 };
    % Boundary constraints
    cbnd = {initial({x == 0; y == 0})
        final({x == 1.2; y == 1.6})};
    % ODEs and path constraints
    ode = collocate({
        dot(x) == vx == V*cos(theta)
        dot(y) == vy == V*sin(theta)
        });
    % A 30th order polynomial is more than sufficient to give good
    % accuracy. However, that means that mcollocate would only check
    % about 60 points. In order to make sure we don’t hit an obstacle,
    % we check 300 evenly spaced points instead, using atPoints.
    obstacles = atPoints(linspace(0,t_f,300), {
        (x-0.4)^2 + (y-0.5)^2 >= 0.1
        (x-0.8)^2 + (y-1.5)^2 >= 0.1});
    % Objective: minimum energy.
    objective = integrate(dot(vx)^2+dot(vy)^2);
    
    options = struct;
    options.name = 'Obstacle avoidance';
    constr = {cbox, cbnd, ode, obstacles};
    solution = ezsolve(objective, constr, x0, options);
    % Optimal x, y, and speed, to use as starting guess in the next iteration
    xopt = subs(x, solution);
    yopt = subs(y, solution);
    vxopt = subs(vx, solution);
    vyopt = subs(vy, solution);
    thetaopt = subs(theta, solution);
    speed = subs(V,solution);
end

%%
figure(1)
th = linspace(0,2*pi,500);
x1 = sqrt(0.1)*cos(th)+0.4;
y1 = sqrt(0.1)*sin(th)+0.5;
x2 = sqrt(0.1)*cos(th)+0.8;
y2 = sqrt(0.1)*sin(th)+1.5;
ezplot(x,y);
hold on
plot(x1,y1,'r',x2,y2,'r');
hold off
xlabel('x');
ylabel('y');
title(sprintf('Obstacle avoidance state variables, Speed = %2.4g',speed));
axis image

