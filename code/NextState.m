function new_config = NextState(curr_config, curr_speeds, delta_t, max_speed)

% Takes curr_config: current configuration of robot
%                    (phi,x,y,J1,J2,J3,J4,J5,W1,W2,W3,W4)
%       curr_speeds: current speeds of the robot arms and wheels (5 joint
%                    speeds thetadot, 4 wheel speeds u) 
%       delta_t: timestep
%       max_speed: speed limit, speed must be between -max_speed and
%                  max_speed
% Returns new_config: new configuration of robot 
%                     (phi,x,y,J1,J2,J3,J4,J5,W1,W2,W3,W4)
% Example Inputs:
% 
% curr_config = zeros(12,1);
% curr_speeds = [0; 0; 0; 0; 0; -10; 10; 10; -10];
% delta_t = 0.01;
% max_speed = 15;
% new_config = NextState(curr_config, curr_speeds, delta_t, max_speed);
% 
% Output:
% new_config =
% 
%     1.2338
%     0.0000
%    -0.0000
%          0
%          0
%          0
%          0
%          0
%    -0.1000
%     0.1000
%     0.1000
%    -0.1000

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Separating config vector into chasis, arm, and wheel components:

curr_chasis_config = curr_config(1:3);   % aka the variable q
curr_arm_config = curr_config(4:8);      % aka the variable theta
curr_wheels_config = curr_config(9:12);  % aka the variable theta

%% Limiting speeds to the value of max_speed:

for i = 1: 1: size(curr_speeds)
    if curr_speeds(i) > max_speed
        curr_speeds(i) = max_speed;
    end
    if curr_speeds(i) < -max_speed
        curr_speeds(i) = -max_speed;
    end
end

%% Separating speed vector into arm and wheel components:

curr_arm_speeds = curr_speeds(1:5);      % aka the variable thetadot
curr_wheel_speeds = curr_speeds(6:9);    % aka the variable u

%% Euler stepping arm and wheel configs:

new_arm_config = curr_arm_config + delta_t * curr_arm_speeds;
new_wheels_config = curr_wheels_config + delta_t * curr_wheel_speeds;

%% Calculating new chasis config according to odometry steps from 13.4:

r = 0.0475; 
l = 0.47/2;
w = 0.3/2;

Vb = r/4 * [-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w);1 1 1 1;-1 1 -1 1] * curr_wheel_speeds;
Vb6 = [0; 0; Vb(1); Vb(2); Vb(3); 0];  % Vb(1) = wbz, Vb(2) = vbx, Vb(3) = vby
%Tbb_prime = MatrixExp6(VecTose3(Vb6));

if Vb6(3) == 0  
    delta_qb = [0; Vb6(4); Vb6(5)];
else
    delta_qb = [Vb6(3); (Vb6(4)*sin(Vb6(3))+Vb6(5)*(cos(Vb6(3))-1))/Vb6(3); ...
        (Vb6(5)*sin(Vb6(3))+Vb6(4)*(1-cos(Vb6(3))))/Vb6(3)]; 
end

delta_q = [1 0 0; 0 cos(curr_chasis_config(1)) -sin(curr_chasis_config(1)); ...
    0 sin(curr_chasis_config(1)) cos(curr_chasis_config(1))] * delta_qb;

new_chasis_config = curr_chasis_config + delta_q;

%% Joining chasis, arm, and wheel configs:

new_config = [new_chasis_config; new_arm_config; new_wheels_config];  

end