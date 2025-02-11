
function u_best = controller_mpc(proxmity_filter_i,line_vel_i,rotate_vel_i,line_vels_j,rotate_vels_j,value_i,pose_i,theta_i,poses_j,thetas_j,sample_time,distance,lambda,c,p,target_w,goal,simConst)
    %% build cost function and solve with mpc
    nx = 5;
    ny = 5;
    nu = 2;
    mpc_distributed_controller = nlmpc(nx,ny,nu);
    mpc_distributed_controller.Ts = sample_time;
    mpc_distributed_controller.PredictionHorizon = 10;
    mpc_distributed_controller.ControlHorizon = 10;
    
    
    
    function z = myStateFunction(x,u,Ts,goal,proxmity_filter_i,line_vel_i,rotate_vel_i,line_vels_j,rotate_vels_j,value_i,pose_i,theta_i,poses_j,thetas_j,simConst)
       z = zeros(5,1);
       z(1) = x(1) + Ts*(x(4)+u(1)*Ts)*cos(x(3)+Ts*(x(5)+u(2)*Ts));
       z(2) = x(2) + Ts*(x(4)+u(1)*Ts)*sin(x(3)+Ts*(x(5)+u(2)*Ts));
       z(3) = x(3)+Ts*(x(5)+u(2)*Ts);
       z(4) = x(4)+u(1)*Ts;
       z(5) = x(5)+u(2)*Ts;
       %disp('states x')
       %disp(z(1))
       %disp('states y')
       %disp(z(2))
       %disp('u')
       %disp(u)
    end

    function y = myOutputFunction(x,u,Ts,goal,proxmity_filter_i,line_vel_i,rotate_vel_i,line_vels_j,rotate_vels_j,value_i,pose_i,theta_i,poses_j,thetas_j,simConst)
       y = zeros(5,1);
       y(1) = x(1);
       y(2) = x(2);
       y(3) = x(3);
       y(4) = x(4);
       y(5) = x(5);
    end
    
    mpc_distributed_controller.Model.StateFcn = @myStateFunction;
    mpc_distributed_controller.Model.OutputFcn = @myOutputFunction;
    mpc_distributed_controller.Model.IsContinuousTime = false;
    mpc_distributed_controller.Model.NumberOfParameters = 13;
    function J = myCostFunction(X,U,e,data,sample_time,goal,proxmity_filter_i,line_vel_i,rotate_vel_i,line_vels_j,rotate_vels_j,value_i,pose_i,theta_i,poses_j,thetas_j,simConst)
        p = data.PredictionHorizon;
        X_pose_j = zeros(p,1);
        Y_pose_j = zeros(p,1);
        for i=1:p
            
        end
        
        U1 = U(1:p,data.MVIndex(1));
        U2 = U(1:p,data.MVIndex(2));
        X1 = X(2:p+1,1);
        X2 = X(2:p+1,2);
        %X3 = X(2:p+1,3);
        %X4 = X(2:p+1,4);
        %X5 = X(2:p+1,5);
        %X1 = X(p+1,1);
        %X2 = X(p+1,1);
        J = simConst.target_weight* sum((X1-goal(1)).^2+(X2-goal(2)).^2) ...
            + sum(simConst.lambda*(U1.^2+U2.^2)) ...
            + proxmity_filter_i(1)*(X_pose_i-poses_j(1))
        
            + proxmity_filter_i(1)*(norm((pose_i - poses_j(1) + sample_time*(line_vel_i+x(1)*sample_time).* [cos(theta_i+rotate_vel_i*sample_time+x(2)*sample_time);sin(theta_i+rotate_vel_i*sample_time+x(2)*sample_time)] ...
     - sample_time*(line_vels_j(1)).*[cos(thetas_j(1)+rotate_vels_j(1)*sample_time);sin(thetas_j(1)+rotate_vels_j(1)*sample_time)]))-distance)^2 ...
     +proxmity_filter_i(2)*(norm((pose_i - poses_j(2) + sample_time*(line_vel_i+x(1)*sample_time).* [cos(theta_i+rotate_vel_i*sample_time+x(2)*sample_time);sin(theta_i+rotate_vel_i*sample_time+x(2)*sample_time)] ...
     - sample_time*(line_vels_j(2)).*[cos(thetas_j(2)+rotate_vels_j(2)*sample_time);sin(thetas_j(2)+rotate_vels_j(2)*sample_time)]))-distance)^2 ...
     +proxmity_filter_i(3)*(norm((pose_i - poses_j(3) + sample_time*(line_vel_i+x(1)*sample_time).* [cos(theta_i+rotate_vel_i*sample_time+x(2)*sample_time);sin(theta_i+rotate_vel_i*sample_time+x(2)*sample_time)] ...
     - sample_time*(line_vels_j(3)).*[cos(thetas_j(3)+rotate_vels_j(3)*sample_time);sin(thetas_j(3)+rotate_vels_j(3)*sample_time)]))-distance)^2 ...
     +proxmity_filter_i(4)*(norm((pose_i - poses_j(4) + sample_time*(line_vel_i+x(1)*sample_time).* [cos(theta_i+rotate_vel_i*sample_time+x(2)*sample_time);sin(theta_i+rotate_vel_i*sample_time+x(2)*sample_time)] ...
     - sample_time*(line_vels_j(4)).*[cos(thetas_j(4)+rotate_vels_j(4)*sample_time);sin(thetas_j(4)+rotate_vels_j(4)*sample_time)]))-distance)^2
        %J = (X1-goal(1))^2+(X2-goal(2))^2;
%        disp('U1');
%        disp(U1);
%        disp('U2');
%        disp(U2);
%        disp('X1');
%        disp(X1);
%        disp('X2');
%        disp(X2);

    end

    mpc_distributed_controller.Optimization.CustomCostFcn = @myCostFunction;
    x0 = [pose_i(1);pose_i(2);theta_i;line_vel_i;rotate_vel_i];
    u0 = [0;0];

    mpc_distributed_controller.States(4).Min = simConst.min_v_l;
    mpc_distributed_controller.States(4).Max = simConst.max_v_l;
    mpc_distributed_controller.States(5).Min = simConst.min_v_r;
    mpc_distributed_controller.States(5).Max = simConst.max_v_r;
    mpc_distributed_controller.ManipulatedVariables(1).Min = simConst.min_a_l;
    mpc_distributed_controller.ManipulatedVariables(1).Max = simConst.max_a_l;
    mpc_distributed_controller.ManipulatedVariables(2).Min = simConst.min_a_r;
    mpc_distributed_controller.ManipulatedVariables(2).Max = simConst.max_a_r;
    
    validateFcns(mpc_distributed_controller,x0,u0,[],{sample_time,goal,proxmity_filter_i,line_vel_i,rotate_vel_i,line_vels_j,rotate_vels_j,value_i,pose_i,theta_i,poses_j,thetas_j,simConst});
    options = nlmpcmoveopt;
    options.Parameters = {sample_time,goal,proxmity_filter_i,line_vel_i,rotate_vel_i,line_vels_j,rotate_vels_j,value_i,pose_i,theta_i,poses_j,thetas_j,simConst};
    [u_best,~,info] = nlmpcmove(mpc_distributed_controller,x0,u0,[],[],options);
    %u_best = 0;
    %disp(info);
end


