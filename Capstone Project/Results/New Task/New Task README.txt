type of controller: feedforward-plus-PI
Kp = 1.7
Ki = 0.8
Tsc_initial = [1 0 0 1; 0 1 0 2; 0 0 1 0.025; 0 0 0 1];
Tsc_final = [0 1 0 0; -1 0 0 0; 0 0 1 0.025; 0 0 0 1];
Changed (x,y) initial position from (1,0) to (1,2) and final position from (0,-1) to (0,0).
Note: increased N5 from round(5*k/0.01) to round(10*k/0.01) in order to increase the time of the fifth segment trajectory (that takes cube from standoff 1 to standoff 2) since the distance the cube needs to cover is longer.