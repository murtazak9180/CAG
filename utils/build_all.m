% function build_all(params)
%     %% Build cost_sum
%     cost_name = 'cost_sum_SS';
%     U_type = coder.typeof(zeros(2 * params.n * params.h, 1));
%     X_type = coder.typeof(zeros(4 * params.n, 1)); % Initial state
%     params_type = coder.typeof(params);
%     cost_args = {U_type, X_type, params_type};
% 
%     build_codegen(cost_name, cost_args, params.n, params.h)
% 
%     %% Build constraints
%     constraints_name = 'constraints_SS';
%     constraints_args = {U_type, params_type};
%     build_codegen(constraints_name, constraints_args, params.n, params.h);
% end

function build_all(params, method)
    n = params.n;
    h = params.h;

    switch upper(method)
        case 'SS'
            cost_name        = 'cost_sum_SS';
            constraints_name = 'constraints_SS';
            Z_type           = coder.typeof(zeros(2 * n * h, 1));
        case 'MS'
            cost_name        = 'cost_sum_MS';
            constraints_name = 'constraints_MS';
            Z_type           = coder.typeof(zeros(6 * n * h, 1));
        otherwise
            error('Unknown method. Use ''SS'' or ''MS''.');
    end

    X_type      = coder.typeof(zeros(4 * n, 1));
    params_type = coder.typeof(params);
    args        = {Z_type, X_type, params_type};

    build_codegen(cost_name,        args, n, h);
    build_codegen(constraints_name, args, n, h);
end