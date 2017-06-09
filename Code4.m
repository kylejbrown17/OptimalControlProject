%Get creates map 
K = 0.035;
s = [0      20    40   60    80];
k = [K   -2*K   -K   2*K    0];
path = generate_path(s,k,[0;0;0]);
s_end = path.s_m(end); 


%Vehicle parameters
m = 1600;                       % mass in kg
Iz = 2200;                      % moment of inertia in kg m^2
wf = 0.577;                     % weight distribution at front axle
wr = 1-wf;                      % weight distribution at rear axle
veh.L = 2.4;                    % wheelbase
veh.a = wr*veh.L;               % distance from reference point to front axle
veh.b = wf*veh.L;               % distance from reference point to rear axle
veh.rW = 0.35;                  % tire radius
% veh.h1 = 0.4;                   %vertical ditance fom roll axis to cg
% veh.hf = 0.1;                   %front roll center height
% veh.hr = 0.15;                  %rear roll center height
% veh.tf = 1.555;                 %front track width; 
% veh.tr = 1.546;                 %rear track width; 
% Caf = 188*10^3;                 % cornering stiffness of front wheel in N
% Car = 203*10^3;                 % cornering stiffness of rear wheel in N
% K_phif = 81000;                 %Front roll stiffness
% K_phir = 59000;                 %Rear roll stiffness; 
% frr = 0.0157;                 % rolling resistance
% C_DA = 0.594;                 % drag coefficient in m^2 at rho = 1.225 kg/m^3

g = 9.81;                        % gravity
Ux = 8;  

% constant radius  path
% "undulating" path
s_end = path.s_m(end); 
dT = 0.01; 
Tfinal = 1000; 


% generate A, B matrices
R = 4; 
N = 15; 
n =3; m = 1; 
xbar = [1; 0.5]; ubar = 1; 
x0 = [0; 0; 0]; 

% objective and constraints
xmax = xbar; xmin = -xbar; 
umax = ubar; umin = -ubar;
z = x0; optvalfha = zeros(N,1);

% model predictive control
Xallmpc = zeros(n,Tfinal); Uallmpc = zeros(m,Tfinal); Sall = zeros(1, Tfinal);
T = N; 
% for T = minT:N
    x = z;
    
    for i = 1:Tfinal
%         cvx_precision(max(min(abs(x))/10,1e-6))
            K = interp1(path.s_m,path.k_1pm, x(1));
            
            
            A = [0 K*Ux 0; 0  0 Ux; 0 -K^2*Ux 0]; B = [0; 0; Ux/veh.L];
            ff = [Ux; 0; -K*Ux]; 
            
            
            cvx_begin quiet
            variables X(n,T+1) U(m,T) S(1,T+1)
            
%           %state constraints
            max(X(2,:)) <= xmax(1); min(X(2,:)) >= xmin(1);
%             max(X(3,:)) <= xmax(2); min(X(3,:)) >= xmin(2)
            max(X(1,:)) <= s_end; 
            
            %control constraints
            max(U') <= umax'; min(U') >= umin';
       
            %System dynamics
            X(:, 2:T+1) ==  X(:, 1:T) + dT*(A*X(:,1:T) + B*U(1:T) + ff*(ones(1, T))); 
            X(:,1) == x;  
            S == s_end*ones(1, T+1) - X(1, :); 
            
            %control objectives
            minimize (1); 
            cvx_end
           
           %Receding horizon implementation
           u = U(:,1);

           Xallmpc(:,i) = x; 
           Uallmpc(:,i) = u;
          
           %Propagate dynamics forward into next state
           x = x + dT*(A*x + B*u + ff) ; 
           
        if strcmp(cvx_status, 'Infeasible') 
            Qhalf
            break; 
        end;
        if strcmp(cvx_status, 'Unbounded')
            Rhalf
            break;         
        end; 
                
    end
    
   t = 0:dT:(Tfinal-1)*dT;
   
   
   figure;
   plot(t, Xallmpc(2:n, :)); 
   s_m = Xallmpc(1,:);  
   e_m = Xallmpc(2,:); 
   dpsi_rad = Xallmpc(3,:); 
  
   figure; 
   plot(t, Uallmpc); 
   delta_rad = Uallmpc; 
   
   animate(path, veh, dpsi_rad, s_m, e_m, delta_rad);

%     
% figure;
% plot(Xallmpc(1,:), Xallmpc(2,:)); 
% title('Closed-loop Trajectory, x_0 = [-4.5; 2]'); 
% xlabel('x_1');
% ylabel('x_2'); 
