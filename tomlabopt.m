%% AA203 - Optimal Autonomous Racing
% PROPT with Tomlab Models
%
%% Car Parameters
% Sizing
lf = 1.8;    % m, distance from CM to front axle
w = 3.4;     % m, wheelbase
lr = w - lf; % m, distance from CM to rear axle
% Speed
V = 5; % m/s

%% Read Track Parameters
[x_track,y_track,th_track,k_track,s_track,n_track] = readTrack();

x_track = x_track(1:10);
y_track = y_track(1:10);
th_track = th_track(1:10);
k_track = k_track(1:10);
s_track = s_track(1:10);
n_track = n_track(1:10);

%% Reparameterization wrt Arclength
s0 = s_track(1); 
sf = s_track(end);

%% Define model with collocation points
toms s
p = tomPhase('p', s, s0, sf, 40);
setPhase(p);

tomStates x y psi n
tomControls beta

% Set initial guess to center line
x0 = collocate({x == interp1(s_track,x_track,s)';
    y == interp1(s_track,y_track,s)';
    psi == interp1(s_track,th_track,s)';
    n == 0});

%% Define constraints
% Box constraints
cbox = collocate({-2*pi <= psi <= 2*pi;
    -interp1(s_track,n_track,s) <= n <= interp1(s_track,n_track,s)});

% Boundary constraints
cbnd = {initial({x == x_track(1); y == y_track(1); psi == th_track(1); n == 0})
    final({x == x_track(end); y == y_track(end)})};

%% ODEs and path constraints
ceq = collocate({...
    dot(x) == V*cos(psi+beta) * (1-interp1(s_track,n_track,s)*interp1(s_track,k_track,s)) / (V*cos(psi+beta)*cos(psi-interp1(s_track,th_track,s)) - V*sin(psi+beta)*sin(psi-interp1(s_track,th_track,s)));    
    dot(y) == V*sin(psi+beta) * (1-interp1(s_track,n_track,s)*interp1(s_track,k_track,s)) / (V*cos(psi+beta)*cos(psi-interp1(s_track,th_track,s)) - V*sin(psi+beta)*sin(psi-interp1(s_track,th_track,s)));
    dot(psi) == V/lr*sin(beta) * (1-interp1(s_track,n_track,s)*interp1(s_track,k_track,s)) / (V*cos(psi+beta)*cos(psi-interp1(s_track,th_track,s)) - V*sin(psi+beta)*sin(psi-interp1(s_track,th_track,s)));
    dot(n) == (V*cos(psi+beta)*sin(psi-interp1(s_track,th_track,s))+V*sin(psi+beta)*cos(psi-interp1(s_track,th_track,s))) * (1-interp1(s_track,n_track,s)*interp1(s_track,k_track,s)) / (V*cos(psi+beta)*cos(psi-interp1(s_track,th_track,s)) - V*sin(psi+beta)*sin(psi-interp1(s_track,th_track,s)))    
    });
    
objective = 1/dot(s);

%% Solve!
options = struct;
options.name = 'SpeedRacer';
options.solver = 'knitro';
solution = ezsolve(objective, {cbox, cbnd, ceq}, x0, options);

%% Example
run_ex = false;
if run_ex
    toms t
    toms t_f
    p = tomPhase('p', t, 0, t_f, 30);
    setPhase(p);
    tomStates x1 x2 x3 x4
    tomControls u
    % Initial guess
    x0 = {t_f == 1
        icollocate({
        x1 == 12*t/t_f
        x2 == 45*t/t_f
        x3 == 5*t/t_f
        x4 == 0})};
    % Box constraints
    cbox = {sqrt(eps) <= t_f
        -pi/2 <= collocate(u) <= pi/2};
    % Boundary constraints
    cbnd = {initial({x1 == 0; x2 == 0; x3 == 0; x4 == 0})
        final({x2 == 45; x3 == 5; x4 == 0})};
    % ODEs and path constraints
    a = 100;
    ceq = collocate({dot(x1) == x2
        dot(x2) == a*cos(u)
        dot(x3) == x4
        dot(x4) == a*sin(u)});
    % Objective
    objective = t_f;
    
    options = struct;
    options.name = 'Linear Tangent Steering';
    options.solver = 'knitro';
    solution = ezsolve(objective, {cbox, cbnd, ceq}, x0, options);
end
