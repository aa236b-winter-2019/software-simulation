function qhat = qmult(q)
% Quaternion Hat Operator
% Convert Quaternion into a 4x4 Square Matrix
v = q(1:3);
s = q(4);
hat_v = [0, -v(3), v(2)
         v(3), 0, -v(1) 
		-v(2), v(1), 0];
qhat = [s*eye(3) + hat_v  v ; -v' s]; 
end