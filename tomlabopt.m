%% AA203 - Optimal Autonomous Racing
% PROPT with Tomlab Models
%
%% Read Track Parameters
fname = 'track.csv';

delimiter = ','; startRow = 2;
formatSpec = '%f%f%f%f%f%[^\n\r]';
fileID = fopen(fname,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false);
fclose(fileID);

% Track parameters
x_track = dataArray{:, 1};
y_track = dataArray{:, 2};
th_track = dataArray{:, 3}; % track heading, rad
k_track = dataArray{:, 4}; % curvature
s_track = dataArray{:, 5}; % arclength
N = 10; % track width, m

%% Car Parameters
% Sizing
lf = 1.8; % m, distance from CM to front axle
w = 3.4; % m, wheelbase
lr = w - lf; % m, distance from CM to rear axle
% Speed
V = 5; % m/s

%% Define model with collocation points
toms t
toms tf
ncolloc = numel(s_track);
p = tomPhase('p', t, 0, tf, ncolloc);
setPhase(p);

tomStates x y psi n
tomControls beta

% Set initial guess to finite horizon trajectory
x0 = {tf == sim;
    icollocate(x == interp1(tsim,xsim,t)');
    icollocate(y == interp1(tsim,ysim,t)');
    icollocate(psi == interp1(tsim,psisim,t)');
    icollocate(n == interp1(tsim,nsim,t)')};

%% Define constraints
% Box constraints
cbox = collocate({1.5*min(x_track) <= icollocate(x) <= 1.5*max(x_track)
    1.5*min(y_track) <= icollocate(y) <= 1.5*max(y_track)
    -2*pi <= psi <= 2*pi
    -N <= n <= N});

% Boundary constraints
cbnd = {initial({x == x_track(1); y == y_track(1); psi == th_track(1); n == 0})
    final({x == x_track(end); y == y_track(end);})};

%% ODEs and path constraints
ceq = collocate({dot(x) == V*cos(psi + beta)
    dot(y) == V*sin(psi + beta)
    dot(psi) == V/lr*sin(beta)
    dot(n) == dot(x)*cos(psi) - dot(y)*sin(psi)});
    
    %dot(n) == dot(x)*cos(psi - th_track) - dot(y)*sin(psi - th_track)});

objective = tf;

%% Solve!
options = struct;
options.name = 'SpeedRacer';
options.solver = 'knitro';
solution = ezsolve(objective, {cbox, cbnd, ceq}, x0, options); % cbox
t = subs(collocate(t),solution);

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
