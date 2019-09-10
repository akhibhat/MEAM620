function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0
T = 10;

if t<T/4
    pos = [t/10; (sqrt(2)*t)/2.5; (sqrt(2)*t)/2.5];
    vel = [1/10; sqrt(2)/2.5; sqrt(2)/2.5];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
% elseif t>= 0.1 && t<T/4
%     pos = [t/12; (sqrt(2)*t)/3; (sqrt(2)*t)/3];
%     vel = [0; 0; 0];
%     acc = [0; 0; 0];
%     yaw = 0;
%     yawdot = 0;
    
elseif t>=T/4 && t<T/2
    pos = [t/10; (2*sqrt(2) - (sqrt(2)*t)/2.5); (sqrt(2)*t)/2.5];
    vel = [1/10; -sqrt(2)/2.5; sqrt(2)/2.5];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
elseif t>=T/2 && t<3*T/4
    pos = [t/10; (2*sqrt(2) - (sqrt(2)*t)/2.5); (4*sqrt(2) - (sqrt(2)*t)/2.5)];
    vel = [1/10; -sqrt(2)/2.5; -sqrt(2)/2.5];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
elseif t>=3*T/4 && t<T
    pos = [t/10; (-4*sqrt(2)+(sqrt(2)*t)/2.5); (4*sqrt(2) - (sqrt(2)*t/2.5))];
    vel = [1/10; sqrt(2)/2.5; -sqrt(2)/2.5];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
    
else
    pos = [1;0;0];
    vel = [0;0;0];
    acc = [0; 0; 0];
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
