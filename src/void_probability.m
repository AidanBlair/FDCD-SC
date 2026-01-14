function psi = void_probability(sensor_position, u, step_size, radius, pi_tilde)
    psi = 1;
    for i = 1:size(pi_tilde, 1)
        psi = psi * (1 - pi_tilde{i}.r * (indicator(sensor_position, u, step_size, pi_tilde{i}.x, radius) * pi_tilde{i}.w));
    end
end

function I = indicator(sensor_position, u, step_size, x, radius)
    x_ = x([1,3], :);
    if u == 4
        sensor_position(1) = sensor_position(1) + step_size;
    elseif u == 5
        sensor_position(2) = sensor_position(2) + step_size;
    elseif u == 6
        sensor_position(1) = sensor_position(1) - step_size;
    elseif u == 7
        sensor_position(2) = sensor_position(2) - step_size;
    end
    I = vecnorm(sensor_position - x_) < radius;
end