function est_next_state = propagate_state(state, model, u_ego, dt)
    est_next_state = state;
    % update the dynamics of the tracked quad
    est_next_state(1:13) = step_quad_dynamics(state(1:13), model, [], dt);
    % update our ego dynamics
    est_next_state(14:26) = step_quad_dynamics(state(14:26), model, u_ego, dt);
end
