classdef DiffBehaviorBased_MPC < control
    %DIFFDRIVEBEHAVIORBASED 
    % behavior based controller for robot to navigate 
    % differential drive dynamics
    % including the following behaviors:
    % Go to Goal
    
    properties
        fsm
        goal
        behaviors
        numBehaviors
        controllers
        sensors
        numRobots
        simConst
        robotInfo
    end
    
    methods
        function obj = DiffBehaviorBased_MPC(robotInfo,goal,simConst)
            %DIFFDRIVEBEHAVIORBASED 
            % contruct a behavior-based controller
            obj.simConst = simConst;
            obj.robotInfo = robotInfo;
            valid_dynamics = ["DiffDrive"];
            if (~ismember(robotInfo.type,valid_dynamics))
                msg = "Behavior-based controller: wrong dynamics type";
                error(msg);
            end
            obj.goal = goal;
            obj.behaviors = {'go-to-goal';'avoid-wall'};
            obj.numBehaviors = size(obj.behaviors,1);
            obj.controllers = cell(1,obj.numBehaviors);
            obj.sensors = RangeFinders(robotInfo);
            pursuitInfo = PursuitInfo(0.35,0.75,1.5);
            for i = 1:obj.numBehaviors
                if (obj.behaviors(i,:) == "go-to-goal")
                    obj.controllers{i} = DiffDriveGoToGoal(robotInfo,pursuitInfo,goal);
                end
                if (obj.behaviors(i,:) == "avoid-wall")
                    obj.controllers{i} = DiffDriveAvoidWall(robotInfo,pursuitInfo);
                end
            end
            % initialize finite state machine
            obj.fsm = BehaviorBasedFSM("go-to-goal");
        end
        
        function control = compute_control(obj, index, pose, vel,raw_reads,proxmity_filter_i,poses,vels)
            % index: the index of robots
            % vel: 3x1 v_x,v_y,v_r
            % pose: 3x1, x,y,theta
            
            %% init
            line_vel_i = sqrt(vel(1)^2+vel(2)^2);
            rotate_vel_i = vel(3);
            line_vels_j = sqrt(vels(1,:).^2 + vels(2,:).^2);
            rotate_vels_j = vels(3,:);
            value_i = obj.robotInfo.value;
            pose_i = pose(1:2);
            theta_i = pose(3);
            poses_j = poses(1:2,:);
            thetas_j = poses(3,:);
            
            %% const read
            sample_time = obj.simConst.sampleTime;
            distance = obj.simConst.d;
            lambda = obj.simConst.lambda;
            c = obj.simConst.c;
            p = obj.simConst.protection;
            target_w = obj.simConst.target_weight;
            goal = obj.goal;
            u_best1 = controller_greedy(proxmity_filter_i,line_vel_i,rotate_vel_i,line_vels_j,rotate_vels_j,value_i,pose_i,theta_i,poses_j,thetas_j,sample_time,distance,lambda,c,p,target_w,goal,obj.simConst);
            u_best2 = controller_mpc(proxmity_filter_i,line_vel_i,rotate_vel_i,line_vels_j,rotate_vels_j,value_i,pose_i,theta_i,poses_j,thetas_j,sample_time,distance,lambda,c,p,target_w,goal,obj.simConst);
            disp('u_best');
            disp(u_best1);
            %disp('u_best2');
            %disp(u_best2);
            u_best = u_best2;
             %((xi + sampleTime*(line_vel_i+alpha*sample_time)*cos(theta_i+rotate_vel_i*sample_time+beta*sample_time))- ...
             %xj(1)- sampleTime*line_vel_j(1)*cos(theta_j(1)+rotate_vel_j(1)*sample_time))  
             %)
            %disp('vels') %[range angle idx]
            %disp(vel)
            %pose = poses(:,i);
            %vel = vels(:,i);
            %u_vest = controller_greedy();
            
            % collect readings from sensors
            reads = obj.sensors.fill_nan(raw_reads);
            direc_avoid = obj.sensors.avoid_wall(reads);
            % check for state switching
            angle_g = pose(3);
            R = [
              cos(angle_g) -sin(angle_g);
              sin(angle_g)  cos(angle_g);
            ];
            vec_g = obj.goal - pose(1:2)';
            direc_goal = R'*vec_g';
            [state,obj.fsm] = obj.fsm.compute_next_state(reads,direc_avoid,direc_goal);
            disp(state);
            %compute control by according to state
            switch (state)
                case "go-to-goal" % Go-To-Goal State
                    controller_idx = find(strcmp(obj.behaviors, state)==1);
                    controller = obj.controllers{controller_idx};
                    control = controller.compute_control(pose,reads);
                case "avoid-wall"
                    controller_idx = find(strcmp(obj.behaviors, state)==1);
                    controller = obj.controllers{controller_idx};
                    control = controller.compute_control(pose,direc_avoid);
                otherwise
                    control.vRef = 0;
                    control.wRef = 0;
            end
            control.vRef = u_best(1)*sample_time + line_vel_i;
            control.wRef = u_best(2)*sample_time + rotate_vel_i;
        end
    end
end

% proxmity_filter_i(1)*(norm((pose_i - poses_j(1) + sample_time*(line_vel_i+x(1)*sample_time).* [cos(theta_i+rotate_vel_i*sample_time+x(2)*sample_time);sin(theta_i+rotate_vel_i*sample_time+x(2)*sample_time)] ...
%                  - sample_time*(line_vels_j(1)).*[cos(thetas_j(1)+rotate_vels_j(1)*sample_time);sin(thetas_j(1)+rotate_vels_j(1)*sample_time)]))-distance)^2 ...
%                  +proxmity_filter_i(2)*(norm((pose_i - poses_j(2) + sample_time*(line_vel_i+x(1)*sample_time).* [cos(theta_i+rotate_vel_i*sample_time+x(2)*sample_time);sin(theta_i+rotate_vel_i*sample_time+x(2)*sample_time)] ...
%                  - sample_time*(line_vels_j(2)).*[cos(thetas_j(2)+rotate_vels_j(2)*sample_time);sin(thetas_j(2)+rotate_vels_j(2)*sample_time)]))-distance)^2 ...
%                  +proxmity_filter_i(3)*(norm((pose_i - poses_j(3) + sample_time*(line_vel_i+x(1)*sample_time).* [cos(theta_i+rotate_vel_i*sample_time+x(2)*sample_time);sin(theta_i+rotate_vel_i*sample_time+x(2)*sample_time)] ...
%                  - sample_time*(line_vels_j(3)).*[cos(thetas_j(3)+rotate_vels_j(3)*sample_time);sin(thetas_j(3)+rotate_vels_j(3)*sample_time)]))-distance)^2 ...
%                  +proxmity_filter_i(4)*(norm((pose_i - poses_j(4) + sample_time*(line_vel_i+x(1)*sample_time).* [cos(theta_i+rotate_vel_i*sample_time+x(2)*sample_time);sin(theta_i+rotate_vel_i*sample_time+x(2)*sample_time)] ...
%                  - sample_time*(line_vels_j(4)).*[cos(thetas_j(4)+rotate_vels_j(4)*sample_time);sin(thetas_j(4)+rotate_vels_j(4)*sample_time)]))-distance)^2 ...
%                  + norm(x) ...
%                  +value_i*proxmity_filter_i(1)*(norm((pose_i - poses_j(1) + sample_time*(line_vel_i+x(1)*sample_time).* [cos(theta_i+rotate_vel_i*sample_time+x(2)*sample_time);sin(theta_i+rotate_vel_i*sample_time+x(2)*sample_time)] ...
%                  - sample_time*(line_vels_j(1)).*[cos(thetas_j(1)+rotate_vels_j(1)*sample_time);sin(thetas_j(1)+rotate_vels_j(1)*sample_time)]))) ...
%                  + value_i*proxmity_filter_i(2)*(norm((pose_i - poses_j(2) + sample_time*(line_vel_i+x(1)*sample_time).* [cos(theta_i+rotate_vel_i*sample_time+x(2)*sample_time);sin(theta_i+rotate_vel_i*sample_time+x(2)*sample_time)] ...
%                  - sample_time*(line_vels_j(2)).*[cos(thetas_j(2)+rotate_vels_j(2)*sample_time);sin(thetas_j(2)+rotate_vels_j(2)*sample_time)]))) ...
%                  + value_i*proxmity_filter_i(3)*(norm((pose_i - poses_j(3) + sample_time*(line_vel_i+x(1)*sample_time).* [cos(theta_i+rotate_vel_i*sample_time+x(2)*sample_time);sin(theta_i+rotate_vel_i*sample_time+x(2)*sample_time)] ...
%                  - sample_time*(line_vels_j(3)).*[cos(thetas_j(3)+rotate_vels_j(3)*sample_time);sin(thetas_j(3)+rotate_vels_j(3)*sample_time)]))) ...
%                  + value_i*proxmity_filter_i(4)*(norm((pose_i - poses_j(4) + sample_time*(line_vel_i+x(1)*sample_time).* [cos(theta_i+rotate_vel_i*sample_time+x(2)*sample_time);sin(theta_i+rotate_vel_i*sample_time+x(2)*sample_time)] ...
%                  - sample_time*(line_vels_j(4)).*[cos(thetas_j(4)+rotate_vels_j(4)*sample_time);sin(thetas_j(4)+rotate_vels_j(4)*sample_time)]))) ...
%                  + 10*norm((pose_i - obj.goal + sample_time*(line_vel_i+x(1)*sample_time).* [cos(theta_i+rotate_vel_i*sample_time+x(2)*sample_time);sin(theta_i+rotate_vel_i*sample_time+x(2)*sample_time)]));

