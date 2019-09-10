function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0
T = 11;
r = 5;

a = 2*pi/T;

if t<9.75
    pos = [r*cos(a*t); r*sin(a*t); 2.5*t/T];
    vel = [-r*a*sin(a*t); r*a*cos(a*t); 2.5/T];
    acc = [-r*a*a*cos(a*t); -r*a*a*sin(a*t); 0];
%     vel = [0;0;0];
%     acc = [0;0;0];
    yaw = 0;
    yawdot = 0;
elseif t>9.75 && t<(T-0.25)
    pos = [r*cos(a*t); r*sin(a*t); 2.5*t/T];
    %vel = [-r*a*sin(a*t);(0.0647*t^3 -2.4411*t^2 + 30.3276*t - 123.5736); 2.5/T];
    %acc = [-r*a*a*cos(a*t);(0.1941*t^2-4.8822*t + 30.3276);0];
    vel = [-r*a*sin(a*t);r*a*cos(a*t); 2.5/T];
    acc = [-r*a*a*cos(a*t);-r*a*a*sin(a*t);0];
%     vel = [0;0;0];
%     acc = [0;0;0];
    yaw = 0;
    yawdot = 0;
else
    pos = [5;0;2.5];
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
