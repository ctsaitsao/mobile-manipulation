function traj = ScrewTrajectory2(Xstart, Xend, Tf, N, method, grip)
%
% Modified ScrewTrajectory to make each row of traj equal to: r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state
%
% Example Input:
% 
% clear; clc;
% Xstart = [[1 ,0, 0, 1]; [0, 1, 0, 0]; [0, 0, 1, 1]; [0, 0, 0, 1]];
% Xend = [[0, 0, 1, 0.1]; [1, 0, 0, 0]; [0, 1, 0, 4.1]; [0, 0, 0, 1]];
% Tf = 5;
% N = 4;
% method = 3;
% traj = ScrewTrajectory2(Xstart, Xend, Tf, N, method, 0)
% 
% Output:
%
% traj =
%   4×1 cell array
% 
%     {1×13 cell}
%     {1×13 cell}
%     {1×13 cell}
%     {1×13 cell}

timegap = Tf / (N - 1);
T = cell(1,N);
traj = cell(N, 13);
for i = 1: N
    if method == 3
        s = CubicTimeScaling(Tf, timegap * (i - 1));
    else
        s = QuinticTimeScaling(Tf, timegap * (i - 1));
    end
    T{i} = Xstart * MatrixExp6(MatrixLog6(TransInv(Xstart) * Xend) * s);
    traj{i,1} = T{i}(1,1); 
    traj{i,2} = T{i}(1,2); 
    traj{i,3} = T{i}(1,3); 
    traj{i,4} = T{i}(2,1); 
    traj{i,5} = T{i}(2,2); 
    traj{i,6} = T{i}(2,3); 
    traj{i,7} = T{i}(3,1); 
    traj{i,8} = T{i}(3,2); 
    traj{i,9} = T{i}(3,3); 
    traj{i,10} = T{i}(1,4); 
    traj{i,11} = T{i}(2,4); 
    traj{i,12} = T{i}(3,4); 
    traj{i,13} = grip;
end
end

