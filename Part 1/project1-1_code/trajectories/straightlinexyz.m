function [desired_state] = straightlinexyz(t, qn)
% HOVER creates hover

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
%    
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

if t<2
    pos = [0; 0; 0];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
elseif t>2 && t<6
    pos = [1/4;sqrt(2);sqrt(2)];
    vel = [0;0;0];
    acc = [0;0;0];
    yaw = 0;
    yawdot = 0;
elseif t>6 && t<10
    pos = [1/2;0;2*sqrt(2)];
    vel = [0;0;0];
    acc = [0;0;0];
    yaw = 0;
    yawdot = 0;

elseif t>10 && t<14
    pos = [3/4;-sqrt(2);sqrt(2)];
    vel = [0;0;0];
    acc = [0;0;0];
    yaw = 0;
    yawdot = 0;
else
    pos = [1;0;0];
    vel = [0;0;0];
    acc = [0;0;0];
    yaw = 0;
    yawdot = 0;
end
% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
