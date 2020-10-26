%% Setting parameters:

curr_config = zeros(12,1);
curr_speeds = [0; 0; 0; 0; 0; -10; 10; 10; -10];
delta_t = 0.01;
max_speed = 15;

%% Initial configs:

new_config = NextState(curr_config, curr_speeds, delta_t, max_speed);
new_config_row = NextStateRow(new_config, 0);
traj = zeros(1,13);

%% Looping to iterate 100 times = 1 second:

for i = 1:1:100
    curr_config = new_config;
    new_config = NextState(curr_config, curr_speeds, delta_t, max_speed);
    new_config_row = NextStateRow(new_config, 0);
    traj = [traj; repmat(new_config_row,1,1)];
end

csvwrite('NextState_test.csv', traj);  