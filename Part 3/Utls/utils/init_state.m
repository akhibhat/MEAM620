function [ s ] = init_state( pos_0, R_0 )
% pos_0: 3x1 vector of initial positions
% R_0:   3x3 rotation matrix of initial orrientation

q_0 = RotToQuat(R_0); % convert R to quaternion

s     = zeros(13,1);  % initialize state vector
s(1)  = pos_0(1); %x
s(2)  = pos_0(2); %y
s(3)  = pos_0(3); %z
s(4)  = 0;        %xdot
s(5)  = 0;        %ydot
s(6)  = 0;        %zdot
s(7)  = q_0(1);   %qw
s(8)  = q_0(2);   %qx
s(9)  = q_0(3);   %qy
s(10) = q_0(4);   %qz
s(11) = 0;        %p
s(12) = 0;        %q
s(13) = 0;        %r

end
