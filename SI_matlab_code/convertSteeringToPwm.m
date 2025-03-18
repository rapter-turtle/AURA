function steer = convertSteeringToPwm(pwm)
    if pwm >= 2000
        % PWM 2000은 steer 300을 의미
        steer = 300;
    elseif pwm > 1500 && pwm < 2000
        % PWM 1500과 2000 사이의 값은 [0, 300] 사이의 steer 값을 의미
        steer = (pwm - 1500) / 1.6667;
    elseif pwm >= 1000 && pwm <= 1500
        % PWM 1000과 1500 사이의 값은 [-300, 0] 사이의 steer 값을 의미
        steer = (pwm - 1500) / 1.6667;
    elseif pwm <= 1000
        % PWM 1000은 steer -300을 의미
        steer = -300;
    else
end

