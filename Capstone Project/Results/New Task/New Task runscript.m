%% Creating Desired Trajectory:

disp('Generating reference trajectory.')
Tse_initial = [0 0 1 0; 0 1 0 0; -1 0 0 0.5; 0 0 0 1]; 
Tsc_initial = [1 0 0 1; 0 1 0 2; 0 0 1 0.025; 0 0 0 1];
Tsc_final = [0 1 0 0; -1 0 0 0; 0 0 1 0.025; 0 0 0 1];
Tce_grasp = [0 0 1 0; 0 1 0 0; -1 0 0 0; 0 0 0 1];
Tce_standoff = [0 0 1 0; 0 1 0 0; -1 0 0 0.03; 0 0 0 1];
k = 20;  % 2000 Hz
traj_desired = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, ...
    Tce_standoff, k);  % aka Tse(t)

%% Initial Values/Parameters:

disp('Inputting initial values and parameters.')
curr_config = [0.352; -0.335; 0.121;...   % aka variable q(t)
    -0.289; -0.120; -0.167; -0.176; ...   % fulfilling the requirement of at least 30 degrees 
    0.459; 0; 0; 0; 0; 0];                % of orientation error and 0.2 m of position error
z = 0.0963;
Kp = 1.7; 
Ki = 0.8; 
r = 0.0475; 
l = 0.47/2;
w = 0.3/2;
F = r/4 * [-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w);1 1 1 1;-1 1 -1 1];
F6 = [0 0 0 0; 0 0 0 0; F; 0 0 0 0];
M0e = [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1];
Blist = [0 0 0 0 0; 0 -1 -1 -1 0; 1 0 0 0 1; 0 -0.5076 -0.3526 -0.2176 0; ...
    0.033 0 0 0 0; 0 0 0 0 0]; 
Tb0 = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1];
delta_t = 0.01;
max_speed = 15;

%% Iterating through Trajectory:

traj_actual = [];
Xerr_list = [];

disp('Iterating through reference trajectory to produce controlled trajectory.')

for i = 1: 1: (size(traj_desired,1)-1)
    
    % Calculating control law to generate wheel and joint controls:
    
    Xd = [traj_desired{i,1} traj_desired{i,2} traj_desired{i,3} traj_desired{i,10}; ...
        traj_desired{i,4} traj_desired{i,5} traj_desired{i,6} traj_desired{i,11}; ...
        traj_desired{i,7} traj_desired{i,8} traj_desired{i,9} traj_desired{i,12}; ...
        0 0 0 1];        % Xd = Tse (from trajectory)
    Xd_next = [traj_desired{i+1,1} traj_desired{i+1,2} traj_desired{i+1,3} traj_desired{i+1,10}; ...
        traj_desired{i+1,4} traj_desired{i+1,5} traj_desired{i+1,6} traj_desired{i+1,11}; ...
        traj_desired{i+1,7} traj_desired{i+1,8} traj_desired{i+1,9} traj_desired{i+1,12}; ...
        0 0 0 1];
    phi = curr_config(1); 
    x = curr_config(2); 
    y = curr_config(3);
    Tsb = [cos(phi) -sin(phi) 0 x; sin(phi) cos(phi) 0 y; 0 0 1 z; 0 0 0 1];
    T0e = FKinBody(M0e, Blist, curr_config(4:8));
    X = Tsb*Tb0*T0e;     % X = Tse (actual)
    Ve = FeedbackControl(X,Xd,Xd_next,Kp,Ki,delta_t);
    Jarm = JacobianBody(Blist, curr_config(4:8));
    Jbase = Adjoint(TransInv(T0e)*TransInv(Tb0))*F6; 
    Je = [Jbase Jarm];
    velocities = pinv(Je)*Ve;
    u = velocities(1:4);
    thetadot = velocities(5:9);
    curr_speeds = [thetadot; u];  % reversing the order of speeds
    
    % Sending controls, configuration, and timestep to NextState:
    
    new_config = NextState(curr_config, curr_speeds, delta_t, max_speed);  
    curr_config = new_config;
    
    % Storing kth configuration:
    
    if mod(i,k) == 0
        traj_actual = [traj_actual; NextStateRow(new_config, traj_desired{i,13})];
    end
    
    % Storing kth Xerr 6-vector:
    
    if mod(i,k) == 0
        Xerr_list = [Xerr_list; se3ToVec(MatrixLog6(TransInv(X)*Xd))'];
    end
    
end

%% Plotting Xerr:

disp('Plotting error over time and outputting as .csv file.')
figure
plot(Xerr_list)
xlabel('Iteration')
ylabel('Error Magnitude (m/s,rad/s)')
grid on
title("Plot of Components of Xerr")
csvwrite('Xerr_list.csv', Xerr_list);

%% Writing to .csv file:

disp('Generating .csv file of controlled trajectory.')
csvwrite('runscript.csv', traj_actual);