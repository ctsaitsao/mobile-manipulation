%% Parameters that are given in Milestone 3:
initial_config = [0; 0; 0; 0; 0; 0.2; -1.6; 0]; 
Kp = eye(6); 
Ki = 0; 
delta_t = 0.01;
Xd = [0 0 1 0.5; 0 1 0 0; -1 0 0 0.5; 0 0 0 1]; 
Xd_next = [0 0 1 0.6; 0 1 0 0; -1 0 0 0.3; 0 0 0 1]; 

%% Calculating Arm Jacobian:
Blist = [0 0 0 0 0; 0 -1 -1 -1 0; 1 0 0 0 1; 0 -0.5076 -0.3526 -0.2176 0; ...
    0.033 0 0 0 0; 0 0 0 0 0]; 
thetalist = initial_config(4:8);
Jarm = JacobianBody(Blist, thetalist);

%% Calculating Base Jacobian:
r = 0.0475; 
l = 0.47/2;
w = 0.3/2;
F = r/4 * [-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w);1 1 1 1;-1 1 -1 1];
F6 = [0 0 0 0; 0 0 0 0; F; 0 0 0 0];
M0e = [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1];
T0e = FKinBody(M0e, Blist, thetalist);
Tb0 = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1];
Jbase = Adjoint(TransInv(T0e)*TransInv(Tb0))*F6; 

%% Calculating Combined Jacobian:
Je = [Jbase Jarm]

%% Calculating X:
phi = initial_config(1);
x =initial_config(2);
y = initial_config(3);
z = 0.0963;
Tsb = [cos(phi) -sin(phi) 0 x; sin(phi) cos(phi) 0 y; 0 0 1 z; 0 0 0 1]; 
X = Tsb*Tb0*T0e; 

%% Calculating Ve:
Ve = FeedbackControl(X,Xd,Xd_next,Kp,Ki,0.01); 

%% Calculating Wheel and Joint Velocities:
velocities = pinv(Je)*Ve;
u = velocities(1:4)
thetadot = velocities(5:9)