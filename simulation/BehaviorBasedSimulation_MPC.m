classdef BehaviorBasedSimulation_MPC < simulation
    %BEHAVIORBASEDSIMULATION 
    % A simulation for robots navigation using behavior-based method
    
    properties
    end
    
    methods
        function obj = BehaviorBasedSimulation_MPC(map,swarmInfo,simConst)
            %BEHAVIORBASEDSIMULATION 
            % construct the simulation
            obj.simConst = simConst;
            obj.sampleTime = simConst.sampleTime;
            obj.numRobots = swarmInfo.numRobots;
            obj.world = world(swarmInfo,simConst);
            obj.controllers = cell(1,obj.numRobots);
            % assign controller to each robot
            goal = simConst.goal; 
            robotInfos = swarmInfo.infos;
            for i = 1:obj.numRobots
                robotInfo = robotInfos{i};
                obj.controllers{i} =  DiffBehaviorBased_MPC(robotInfo,goal,simConst) ; %DiffDriveBehaviorBasedBlend(robotInfo,goal); % 
            end
            % assign actuator to each robot
            obj.actuators = cell(1,obj.numRobots);
            for i = 1:obj.numRobots
                robotInfo = swarmInfo.infos{i};
                R = robotInfo.wheel_radius;
                L = robotInfo.body_width;
                if (robotInfo.type == "DiffDrive")
                    obj.actuators{i} = actuatorDiffDrive(R,L);
                end
            end
            % setup environment physics
            obj.physics = AABB(map,swarmInfo.numRobots,0.25,true);
            obj.prev_poses = swarmInfo.poses;
            obj.prev_vels = swarmInfo.vels;
            
        end
        
        function obj = step(obj)
            readings = obj.sensor_phase();
            detections = obj.detection_phase(); %[numDetections, 3] [range angle idx]
            %disp('detections')
            %disp(size(detections))
            %disp(detections(1,1))
            controls = obj.control_phase(readings,detections);
            %poses = obj.actuate_phase_(controls);
            %%%%%%%%%%%%
            poses = obj.world.get_poses();
            vels = obj.actuate_phase_vel(controls);
            for i = 1:obj.numRobots
                pose = poses(:,i);
                poses(:,i) = pose  + vels(:,i) * obj.sampleTime;
            end
            obj = obj.physics_phase(poses,vels);
            obj.visualize_();
        end
        
         function controls = control_phase(obj,readings,detections)
            poses = obj.world.get_poses(); % current system states
            vels = obj.world.get_vels();
            %disp('poses')
            %disp(poses)
            controls = cell(1,obj.numRobots);
            for i = 1:obj.numRobots
                ctl = obj.controllers{i};
                pose = poses(:,i);
                vel = vels(:,i);
                reading = readings{i};
                detection = detections{i};
                proxmity_filter_i = zeros(obj.numRobots,1);
                if ~isempty(detection)
                    [neighbor_num,~] = size(detection);
                    poses_j = [];
                    vels_j = [];
                    for j = 1:neighbor_num
                       neighobr_idx = detection(j,3);
                       proxmity_filter_i(neighobr_idx) = 1;                     
                     %% debug
                        disp('debug')
                        disp('proxmity_filter_i')
                        disp(proxmity_filter_i)
                    end
                else
                end

                controls{i} = ctl.compute_control(i,pose,vel,reading,proxmity_filter_i,poses,vels);
            end
         end
    end
end

