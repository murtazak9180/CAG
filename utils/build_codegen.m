function build_codegen(name, args, n, h)
    % build_codegen - Compiles a specified MATLAB function into a MEX file using codegen
    % Usage:
    %   build_codegen(name, args)
    %
    % Inputs:
    %   name - string, name of the function (without '_m') to compile
    %   args - cell array, code generation types or example inputs for codegen
    % Usama Mehmood - April 2026

    func_name = [name, '_m'];
    out  = fullfile('generated', sprintf('%s_n%d_h%d_mex', name, n, h));

    fprintf('Codegen compiling %s -> %s...\n', func_name, out);

    cfg_mex = coder.config('mex');
    cfg_mex.ConstantInputs = 'Remove';
    cfg_mex.GenerateReport = true;

    tic; % Start timing
    codegen( ...
        '-config', cfg_mex, ...
        func_name, ...
        '-o', out, ...
        '-args', args ...
    );
    elapsed_time = toc; % Stop timing

    fprintf('Codegen completed for %s in %.2f seconds.\n', func_name, elapsed_time);
end