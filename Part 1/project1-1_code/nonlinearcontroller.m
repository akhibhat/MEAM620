function [F, M, trpy, drpy] = nonlinearcontroller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
%% Inputs:
%
% qd{qn}: state and desired state information for quadrotor #qn (qn
%         will be = 1 since we are only flying a single robot)
%
%  qd{qn}.pos, qd{qn}.vel   position and velocity
%  qd{qn}.euler = [roll;pitch;yaw]
%  qd{qn}.omega     angular velocity in body frame
% 
%  qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des  desired position, velocity, accel
%  qd{qn}.yaw_des, qd{qn}.yawdot_des
%
% t: current time
%    
% qn: quadrotor number, should always be 1
%    
% params: various parameters
%  params.I     moment of inertia
%  params.grav  gravitational constant g (9.8...m/s^2)
%  params.mass  mass of robot
%
%% Outputs:
%
% F: total thrust commanded (sum of forces from all rotors)
% M: total torque commanded
% trpy: thrust, roll, pitch, yaw (attitude you want to command!)
% drpy: time derivative of trpy
%
% Using these current and desired states, you have to compute the desired
% controls u, and from there F and M
%

% =================== Your code goes here ===================

%% LINEAR CONTROLLER

% % Get state
% g = params.grav;
% pos = qd{qn}.pos;
% vel = qd{qn}.vel;
% 
% pos_des = qd{qn}.pos_des;
% vel_des = qd{qn}.vel_des;
% acc_des = qd{qn}.acc_des;
% 
% euler = qd{qn}.euler;
% omega = qd{qn}.omega;
% psi_des = qd{qn}.yaw_des;
% psi_d_des = qd{qn}.yawdot_des;
% 
% Kp = [2; 2; 7];
% Kd = [2.25; 2.25; 3]; %2.5 2.5 overdamped
% 
% Kp_a = [1; 1; 1] * 100;
% Kd_a = [1.5; 1; 1] * 10;
% 
% u = zeros(4,1);
% 
% %Calculate r_dot_dot_des
% r_dd_des = acc_des - Kd.*(vel-vel_des) - Kp.*(pos-pos_des);
% 
% u(1) = (r_dd_des(3) + params.grav)*params.mass;
% 
% %Calculate desired phi and theta 
% phi_des = ( r_dd_des(1)*tan(psi_des) - r_dd_des(2) ) / g*( sin(psi_des)*tan(psi_des)+cos(psi_des) );
% theta_des = r_dd_des(1)/g*cos(psi_des) - phi_des*tan(psi_des);
% 
% euler_des = [phi_des; theta_des; psi_des];
% omega_des = [0; 0; psi_d_des];
% 
% 
% u(2:4) = params.I*(-Kp_a.*(euler-euler_des) - Kd_a.*(omega-omega_des));



%% NON-LINEAR CONTROLLER

m = params.mass;
g = params.grav;
pos = qd{qn}.pos;
vel = qd{qn}.vel;

pos_des = qd{qn}.pos_des;
vel_des = qd{qn}.vel_des;
acc_des = qd{qn}.acc_des;

euler = qd{qn}.euler;
omega = qd{qn}.omega;
psi_des = qd{qn}.yaw_des;
psi_d_des = qd{qn}.yawdot_des;
omega_des = [0; 0; psi_d_des];

Kp = [100 100 100]';
Kd = [75 75 40]';

K_R = [2400 0 0; 0 2400 0; 0 0 2500];
K_w = [120 0 0; 0 120 0; 0 0 240];


u = zeros(4,1);

%Calculate r_dot_dot_desired
r_dd_des = acc_des - Kd.*(vel-vel_des) - Kp.*(pos-pos_des);
F_des = m* r_dd_des + [0; 0; m*g];


%Compute u1
R = eulzxy2rotmat(euler);
b3 = R*[0 0 1]';
u(1) = transpose(b3)*F_des;

% Determine R desired
b3_des = F_des/norm(F_des);
a_psi = [cos(euler(3)) sin(euler(3)) 0]';
b2_des = cross(b3_des,a_psi) / norm(cross(b3_des,a_psi));
b1_des = cross(b2_des,b3_des);
R_des = [b1_des b2_des b3_des];

% Find error
e_R = .5*veemap((transpose(R_des)*R - transpose(R)*R_des));
e_R = e_R';
e_w = omega - omega_des;

u(2:4) = params.I*(-K_R*e_R - K_w*e_w);


% ==============================

% Desired roll, pitch and yaw (in rad). In the simulator, those will be *ignored*.
% When you are flying in the lab, they *will* be used (because the platform
% has a built-in attitude controller). Best to fill them in already
% during simulation.

eul = rotmat2eulzxy(R_des);
phi_des = eul(1);
theta_des = eul(2);
psi_des = eul(3);

%
%
%
% u    = zeros(4,1); % control input u, you should fill this in
                  
% Thrust
F    = u(1);       % This should be F = u(1) from the project handout

% Moment
M    = u(2:4);     % note: params.I has the moment of inertia

% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end

%
% ------------------------------------------------------------
%    should you decide to write a geometric controller,
%    the following functions should come in handy
%

function m = eulzxy2rotmat(ang)
    phi   = ang(1);
    theta = ang(2);
    psi   = ang(3);
    
    m = [[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), ...
          cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)];
         [cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi), ...
          sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)];
         [-cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)]];
end

function eul = rotmat2eulzxy(R)
    if R(3,2) < 1
        if R(3,2) > -1
            thetaX = asin(R(3,2));
            thetaZ = atan2(-R(1,2), R(2,2));
            thetaY = atan2(-R(3,1), R(3,3));
        else % R(3,2) == -1
            thetaX = -pi/2;
            thetaZ = -atan2(R(1,3),R(1,1));
            thetaY = 0;
        end
    else % R(3,2) == +1
        thetaX = pi/2;
        thetaZ = atan2(R(1,3),R(1,1));
        thetaY = 0;
    end
    eul = [thetaX, thetaY, thetaZ];
end

function w = veemap(R)
    w = [-R(2,3), R(1,3), -R(1,2)];
end