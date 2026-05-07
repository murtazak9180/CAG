clc 
clear all
close all

%% Load params

Runs = 1;

params_cmpc;



%% IPOPT options
opt = struct;
% 
opt.ipopt.print_level = 0;
%opt.ipopt.output_file = 'ac_ipopt_ms_log_scaled.txt';
%opt.ipopt.file_print_level = 3;   
opt.print_time = false;
opt.ipopt.tol = 1e-8;
opt.ipopt.constr_viol_tol = 1e-8;
opt.ipopt.acceptable_tol = 1e-7;
opt.ipopt.max_iter = 4000;
opt.ipopt.linear_solver = 'mumps';  %linear solver - other options include 'ma27' and 'pardiso' 



%% Initial configuration
[posi, veli] = gen_state_random(params);
display_init_connectivity(posi, veli, params);
X0_ = [posi; veli]';
X0 = X0_(:);


%% Main loop that runs the simulations
tic;
for i = 1:Runs
    traj(i) = run_simulation_cag(X0, params, i, opt);
end
toc;
%% Save output

date_string = datestr(datetime,' [yyyy-mm-dd]');
out_traj_name = ['traj_cag'  date_string];

dir_path = 'results/cag_compare/';

[dest_path, out_traj_name] = create_dir(dir_path, out_traj_name);
mkdir([dest_path '/Results']);

fprintf(['\nOUTPUT:\n' dest_path '\n']);

save([dest_path '/' out_traj_name '.mat'], 'traj');


figure;
for idx = 1:Runs
    displayTraj(traj(idx).x,traj(idx).y,traj(idx).vx,traj(idx).vy)
end
