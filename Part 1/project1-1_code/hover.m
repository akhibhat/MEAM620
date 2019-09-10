function [desired_state] = hover(t, qn)
% HOVER creates hover

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
%    
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

if t<0.1
    pos = [0; 0; 0];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
elseif t>=0.1 && t<1
    pos = [1;0;2];
    vel = [0;0;0];
    acc = [0;0;0];
    yaw = 0;
    yawdot = 0;
else
    pos = [1;0;2];
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
