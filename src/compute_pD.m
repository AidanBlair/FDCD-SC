function pD = compute_pD(model,X,senNo,pims, rho_buffer, theta_buffer)
    arguments
        model
        X
        senNo
        pims
        rho_buffer=0
        theta_buffer=0
    end
    
    if isempty(X)
        pD = [];
        return;
    end
    
    % Initialize
    n = size(X, 2);
    pD = 0.99 * ones(n, 1);
    
    % Parameters
    rho_min = 0;
    rho_max = model.range - rho_buffer;
    theta_max = model.FOV - theta_buffer;
    p_D_max = 0.99;
    k_rho = 1.0;
    k_theta = 20;
    
    % Vectorized distance calculation
    P = X([1 3], :);
    M = model.SenLoc(:, senNo);
    rho = sqrt(sum((P - M).^2, 1));
    
    % Vectorized angle calculation
    range_x = P(1,:) - M(1);
    range_y = P(2,:) - M(2);
    theta = atan2(range_y, range_x);
    
    % Vectorized angle normalization
    theta(theta < -pi) = theta(theta < -pi) + 2*pi;
    theta(theta > pi) = theta(theta > pi) - 2*pi;
    
    % Pre-compute min/max angles (MOVE OUTSIDE LOOP!)
    min_angle = model.Heading - model.FOV;
    max_angle = model.Heading + model.FOV;
    
    % Adjust theta for FOV comparison
    theta_adj = theta;
    theta_adj(theta < min_angle) = theta_adj(theta < min_angle) + 2*pi;
    theta_adj(theta > max_angle) = theta_adj(theta > max_angle) - 2*pi;
    
    % Vectorized heading calculation
    heading = abs(theta_adj - model.Heading);
    heading(heading < pi) = heading(heading < pi) + 2*pi;
    heading(heading > pi) = heading(heading > pi) - 2*pi;
    heading = abs(heading);
    
    % Vectorized conditions
    in_FOV = (theta_adj >= min_angle) & (theta_adj <= max_angle);
    in_range = (rho > rho_min) & (rho < rho_max);
    valid = in_range & in_FOV;
    
    % Set pD to 0 for out of FOV
    pD(~in_FOV) = 0;
    
    % Set pD to 0 for beyond max range
    pD(rho > rho_max) = 0;
    
    % Compute sigmoid for valid points
    if pims
        pD(valid) = 1;
    else
        % Vectorized sigmoid calculation
        pD(valid) = sigmoid_pD(rho(valid), rho_max, k_rho, p_D_max) .* sigmoid_pD(heading(valid), theta_max, k_theta, p_D_max);
    end
    
    pD = pD(:);
end

function y = sigmoid_pD(x, x_max, k_x, y_max)
    % Vectorized sigmoid
    y = y_max ./ (1 + exp(k_x * (x - x_max)));
end

