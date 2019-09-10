function [desired_state] = bettertrajectory_generator(t, qn, varargin)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
% t: A scalar, specifying inquiry time
%
% varargin: variable number of input arguments. In the framework,
% this function will first (and only once!) be called like this:
%
% trajectory_generator([],[], 0, path)
%
% i.e. map = varargin{1} and path = varargin{2}.
%       
% path: A N x 3 matrix where each row is (x, y, z) coordinate of a
% point in the path. N is the total number of points in the path
%
% This is when you compute and store the trajectory.
%
% Later it will be called with only t and qn as an argument, at
% which point you generate the desired state for point t.
%

persistent path1 map blocks xyres i timestamps p_x p_y p_z a_x a_y a_z v_x v_y v_z
if isempty(path1)
    if ~isempty(varargin{2})
        path=[varargin{2}];
        map = varargin{1};
        blocks = map.blocks;
        xyres = map.res_xyz(1);
        path2 = prune_neigh(map,blocks,xyres,path);
        path1 = addingpoints(path2);
        [y,x] = size(path1);
        i=2;
        k=1;
        path_2=path1;
        num_t=size(path_2,1);
        time_cal=diff(path_2);
        maxdist=abs(norm(path_2(end,:)-path_2(1,:)));
        if maxdist>70
            z=0.3;
        else
            z=1;
        end
        time = z*vecnorm(time_cal,2,2);%speed is 0.2m/s
        timestamps=[0;cumsum(time)];
        path_constraint=zeros(4*(num_t-1),3);
        path_constraint(1,:)=path_2(1,:);
        path_constraint(2*(num_t-1),:)=path_2(end,:);
        for i=2:num_t-1
           path_constraint(2*(i-1),:) = path_2(i,:);
           path_constraint(2*(i-1)+1,:) = path_2(i,:);
        end

        mat_1=[];
        t=timestamps;
        num_eqs=4*(num_t-1);
        for i =1:(num_t-1)
            mat_1(2*i-1,1+4*(i-1):4*i)=[t(i)^3, t(i)^2, t(i) 1];
            mat_1(2*i,1+4*(i-1):4*i)=[t(i+1)^3, t(i+1)^2, t(i+1) 1];
        end
        %initial velocity conditions
        mat_1(2*(num_t-1)+1,1:4)=[3*t(1)^2 2*t(1) 1 0];
        mat_1(2*(num_t-1)+2,4*(num_t-2)+1:4*(num_t-1))= [3*t(num_t)^2 2*t(num_t) 1 0]  ;
        %inter velocity conditions
        for i = 2:(num_t-1)
            mat_1(2*num_t+i-1,4*(i-2)+1:4*(i))=[3*t(i)^2 2*t(i) 1 0 -3*t(i)^2 -2*t(i) -1 0]; 
        end
        %inter acceleration conditions
        for i = 2:(num_t-1)
            mat_1(3*num_t+i-3,4*(i-2)+1:4*(i))=[6*t(i) 2 0 0 -6*t(i) -2 0 0]; 
        end
        ew=path_constraint;
        constraints=inv(mat_1)*ew;
        for i =1:num_t-1
        p_x(i,:)=constraints(4*(i-1)+1:4*i,1);
        p_y(i,:)=constraints(4*(i-1)+1:4*i,2);
        p_z(i,:)=constraints(4*(i-1)+1:4*i,3);
        end
        
        
        for i =1:num_t-1
        v_x(i,:)=polyder(constraints(4*(i-1)+1:4*i,1));
        v_y(i,:)=polyder(constraints(4*(i-1)+1:4*i,2));
        v_z(i,:)=polyder(constraints(4*(i-1)+1:4*i,3));
        end
        for i =1:num_t-1
        a_x(i,:)=polyder(v_x(i,:));
        a_y(i,:)=polyder(v_y(i,:));
        a_z(i,:)=polyder(v_z(i,:));
        
        end
        a=1;
        t=0;
    end
end

if (t<=timestamps(end)) && (t>0)
    j=sum(t>timestamps);
        pos=[polyval(p_x(j,:),t);polyval(p_y(j,:),t);polyval(p_z(j,:),t)];
        vel=[polyval(v_x(j,:),t);polyval(v_y(j,:),t);polyval(v_z(j,:),t)];
        acc=[polyval(a_x(j,:),t);polyval(a_y(j,:),t);polyval(a_z(j,:),t)];
        yaw = 0;
        yawdot = 0;
elseif t==0
    pos = path1(1,:);
    vel = [0; 0; 0];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
    a=4;
else
    pos = path1(end,:);
    vel = [0; 0; 0];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
    a=4;
    
end


% use the "persistent" keyword to keep your trajectory around
% inbetween function calls


%
% When called without varargin (isempty(varargin) == true), compute
% and return the desired state here.
%

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
