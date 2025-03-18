% convertThrustToPwm: u1 -> pwm1
function pwm1 = convertThrustToPwm(thrust)
    if thrust <= 1500
        pwm1 = 0; % Any value <= 0 thrust maps to PWM 1000
    else
        pwm1 = (thrust - 1550)/3.9; % Linear conversion
    end
end