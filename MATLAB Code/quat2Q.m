function Q = quat2Q(q)
% Inputs:
%   q - Quaternion (4x1)
% Outputs:
%   Q - Rotation Matrix (3x3)
%

v = q(1:3);
s = q(4);

Q = eye(3) + 2*hat(v)*(s*eye(3) + hat(v));
end