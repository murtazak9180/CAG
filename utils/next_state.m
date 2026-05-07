function [next_pos, next_vel] = next_state(cur_pos, cur_vel, cur_acc, params, cfg)
control_steps = uint8(params.ct / params.dt);

for k = 1:control_steps
    [cur_pos, cur_vel] = dynamics(cur_pos, cur_vel, cur_acc, params, cfg.dynamics_mode);
end
next_pos = cur_pos;
next_vel = cur_vel;
end

