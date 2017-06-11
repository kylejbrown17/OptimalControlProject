%% AA203 - Optimal Autonomous Racing
% PROPT with Tomlab Models
%
close all; clc;
%% Car Parameters
% Sizing
lf = 1.8;    % m, distance from CM to front axle
w = 3.4;     % m, wheelbase
lr = w - lf; % m, distance from CM to rear axle
% Speed
V = 5; % m/s

%% Read Track Parameters
[x_track,y_track,th_track,k_track,s_track,n_track] = readTrack();
step = 1;
endidx = 100;

x_track = x_track(1:step:endidx);
y_track = y_track(1:step:endidx);
th_track = th_track(1:step:endidx);
k_track = k_track(1:step:endidx);
s_track = s_track(1:step:endidx);
n_track = n_track(1:step:endidx);

%% Reparameterization wrt Arclength
s0 = s_track(1); 
sf = s_track(end);
ncolloc = 600;

%% Define model with collocation points
toms s
p = tomPhase('p', s, s0, sf, ncolloc);
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
cbox = collocate({-interp1(s_track,n_track,s) <= n <= interp1(s_track,n_track,s)}); 

% Boundary constraints
cbnd = {initial({x == x_track(1); y == y_track(1); psi == th_track(1); n == 0})
    final({x == x_track(end); y == y_track(end)})};

%% ODEs and path constraints
ceq = collocate({...
    dot(x) == V*cos(psi+beta)/dot(s);    
    dot(y) == V*sin(psi+beta)/dot(s);
    dot(psi) == V/lr*sin(beta)/dot(s);
    dot(n) == (V*cos(psi+beta)*sin(psi-interp1(s_track,th_track,s))+V*sin(psi+beta)*cos(psi-interp1(s_track,th_track,s))) / dot(s);
    });
    
objective = 1/dot(s);

%% Solve!
options = struct;
options.name = 'SpeedRacer';
options.solver = 'knitro';
solution = ezsolve(objective, {cbox, cbnd, ceq}, x0, options);

%% Plot Unsmoothed Results
plot(solution.x_p,solution.y_p,'b-*'); hold on;
plot(x_track,y_track,'r-*');

%% Smooth Results