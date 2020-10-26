function new_config_row = NextStateRow(new_config, gripper)

% Meant to be used straight after using NextState. Converts a configuration
% new_config (output of NextState) into a row vector that includes a
% gripper element.

% Example input:

% new_config = [1.2338; 0.0000; -0.0000; 0; 0; 0; 0; 0; -0.1000; 0.1000;
% 0.1000; -0.1000]
% new_config_row = NextStateRow(new_config, 0)

% Output:

% new_config_row =
% 
%   Columns 1 through 10
% 
%     1.2338         0         0         0         0         0         0         0   -0.1000    0.1000
% 
%   Columns 11 through 13
% 
%     0.1000   -0.1000         0

new_config_row = new_config';
new_config_row = [new_config_row gripper];

end