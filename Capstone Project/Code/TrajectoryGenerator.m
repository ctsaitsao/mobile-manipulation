function traj_sum = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k)

% Calculates a reference trajectory depending on the configurations of the
% end-effector and cube

% Example Inputs:

% Tse_initial = [0 0 1 0; 0 1 0 0; -1 0 0 1; 0 0 0 1];
% Tsc_initial = [1 0 0 1; 0 1 0 0; 0 0 1 0.025; 0 0 0 1];
% Tsc_final = [0 1 0 0; -1 0 0 -1; 0 0 1 0.025; 0 0 0 1];
% Tce_grasp = [0 0 1 0; 0 1 0 0; -1 0 0 0; 0 0 0 1];
% Tce_standoff = [0 0 1 0; 0 1 0 0; -1 0 0 0.03; 0 0 0 1];
% k = 1;
% traj_sum = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k)

% Output: traj_sum, which is a large matrix whose rows are the elements of the
% matrix Tse. It also produces a .csv file called "TrajectoryGenerator.csv"
% which contains all this data.

% This program uses the function ScrewTrajectory2, a mofidied version of ScrewTrajectory.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Segment 1: initial -> standoff 

traj_sum = {};
N1 = round(5*k/0.01);     % make this segment longer to decrease error
Tf1 = 5;
Tse_standoff = Tsc_initial*Tce_standoff;
traj1 = ScrewTrajectory2(Tse_initial, Tse_standoff, Tf1, N1, 5, 0);     
traj_sum  = cat(1, traj_sum, traj1); 

%% Segment 2: standoff -> grasp 

N2 = round(2*k/0.01);  
Tf2 = 3;
Tse_grasp = Tsc_initial*Tce_grasp;
traj2 = ScrewTrajectory2(Tse_standoff, Tse_grasp, Tf2, N2, 5, 0);    
traj_sum  = cat(1, traj_sum, traj2);

%% Segment 3: closing gripper

closed = traj_sum(end,:);
closed{1, 13} = 1;
i = 0;
while i < 64
    traj_sum = cat(1, traj_sum, closed);
    i = i + 1;
end

%% Segment 4: grasp -> standoff

N4 = round(2*k/0.01);  
Tf4 = 3;
traj4 = ScrewTrajectory2(Tse_grasp, Tse_standoff, Tf4, N4, 5, 1);    
traj_sum  = cat(1, traj_sum, traj4);

%% Segment 5: standoff -> standoff2

N5 = round(5*k/0.01);     % make this segment longer to decrease error
Tf5 = 5;
Tse_standoff2 = Tsc_final*Tce_standoff;
traj5 = ScrewTrajectory2(Tse_standoff, Tse_standoff2, Tf5, N5, 5, 1);    
traj_sum  = cat(1, traj_sum, traj5);

%% Segment 6: standoff2 -> ungrasp

N6 = round(2*k/0.01);  
Tf6 = 3;
Tse_ungrasp = Tsc_final*Tce_grasp;
traj6 = ScrewTrajectory2(Tse_standoff2, Tse_ungrasp, Tf6, N6, 5, 1);    
traj_sum  = cat(1, traj_sum, traj6);

%% Segment 7: opening gripper

opened = traj_sum(end,:);
opened{1, 13} = 0;
j = 0;
while j < 64
    traj_sum = cat(1, traj_sum, opened);
    j = j + 1;
end

%% Segment 8: ungrasp -> standoff

N8 = round(2*k/0.01);  
Tf8 = 3;
traj8 = ScrewTrajectory2(Tse_ungrasp, Tse_standoff2, Tf8, N8, 5, 0);    
traj_sum  = cat(1, traj_sum, traj8);

csvwrite('TrajectoryGenerator.csv', traj_sum);  % writing into a csv file that can be read by V-REP

end