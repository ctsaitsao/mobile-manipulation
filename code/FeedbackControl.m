function Ve = FeedbackControl(X, Xd, Xd_next, Kp, Ki, delta_t)

% Calculates an iteration of the feedback control twist that transforms 
% current config to desired config

% Example inputs:

% Kp = eye(6); 
% Ki = 0; 
% delta_t = 0.01;
% Xd = [0 0 1 0.5; 0 1 0 0; -1 0 0 0.5; 0 0 0 1]; 
% Xd_next = [0 0 1 0.6; 0 1 0 0; -1 0 0 0.3; 0 0 0 1]; 
% X = [0.170 0 0.985 0.387; 0 1 0 0; -0.985 0 0.170 0.570; 0 0 0 1];
% Ve = FeedbackControl(X, Xd, Xd_next, Kp, Ki, delta_t)

% Output:

% Ve =
% 
%          0
%     0.1709
%          0
%    21.4795
%          0
%     6.5567

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Kp = Kp*eye(6);  % changing a scalar to an positive-definite matrix
Ki = Ki*eye(6);
Vd = se3ToVec((1/delta_t)*MatrixLog6(TransInv(Xd)*Xd_next));  % finding feedforward reference twist
Xerr = se3ToVec(MatrixLog6(TransInv(X)*Xd));  % finding Xerr, error transformation matrix
Ve = Adjoint(TransInv(X)*Xd)*Vd + Kp*Xerr + Ki*Xerr*delta_t;  % finding twist that transforms 
                                                           % current config to desired config
end