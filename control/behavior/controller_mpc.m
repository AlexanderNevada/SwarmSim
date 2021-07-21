% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   Q: State weighting matrix, dimension (3,3)
%   R: Input weighting matrix, dimension (3,3)
%   T: Measured system temperatures, dimension (3,1) % original values
%   N: MPC horizon length, dimension (1,1)
% OUTPUT:
%   p: Heating and cooling power, dimension (3,1)

function p = controller_mpc_1(Q, R, T, N, ~)
    % controller variables
    persistent param yalmip_optimizer
    
    % initialize controller, if not done already
    if isempty(yalmip_optimizer)
        [param, yalmip_optimizer] = init(Q, R, N);
    end
    
    T=T-param.T_sp;
    % evaluate control action by solving MPC problem
    [u_mpc,errorcode] = yalmip_optimizer(T);
    if (errorcode ~= 0)
        warning('MPC1 infeasible');
    end
    p = u_mpc+param.p_sp; %add delta to output
end

function [param, yalmip_optimizer] = init(Q, R, N)
    % get basic controller parameters
    param = compute_controller_base_parameters;
    A = param.A ;
    B = param.B ;

    % for terminal cost
    [P, K] = idare(A,B,Q,R);

    % implement your MPC using Yalmip here
    nx = size(param.A,2); % dimension check
    nu = size(param.B,2); 

    %sdpvar([x1,x2],[y1,y2]),return cell array
    %e.g.  {x1×y1 sdpvar}  {x2×y2 sdpvar}
    U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full'); %real variable 3*29
    X = sdpvar(repmat(nx,1,N),ones(1,N),'full');  %3*30
    
    objective = 0;
    constraints = [];
    for i = 1:N-1
        % system dynamics constraints
        constraints = [constraints, X{i+1} == param.A * X{i} + param.B * U{i}];
        % state constraints
        constraints = [constraints, param.Ax * X{i+1} <= param.bx];
        % input constraints
        constraints = [constraints, param.Au * U{i} <= param.bu];
        % sum of stage cost function
        objective = objective + X{i}' * Q * X{i} + U{i}' * R * U{i};
    end
    
    %add terminal const.
    objective = objective + X{end}'*P*X{end};
   
    T0 = sdpvar(nx,1); 
    
    constraints = [constraints, X{1} == T0];
    
    ops = sdpsettings('verbose',0,'solver', 'quadprog');
    
    %U{1}:output
    yalmip_optimizer = optimizer(constraints,objective,ops,T0,U{1});
end
