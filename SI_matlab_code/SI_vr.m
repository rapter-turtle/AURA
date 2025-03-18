clc; clear; close all;

% 데이터를 불러옵니다.
data = readtable('030623_1.csv'); % 데이터 파일 경로
psi = data.Var4;
u = data.Var5;
v = data.Var6;
r = data.Var7;
u1 = data.Var14;
u2 = data.Var15;

b1 = 0.00058466;
b2 = 0.0040635;

% 원하는 영역을 설정합니다.
start_idx = 5100; % 시작 인덱스
end_idx = 6000; % 끝 인덱스
dt = 0.1; % 시간 간격
t = (start_idx:end_idx)*dt; % 시간 배열 (선택된 구간)

% 선택된 구간에서 데이터 추출
psi_selected = psi(start_idx:end_idx);
u_selected = u(start_idx:end_idx);
v_selected = v(start_idx:end_idx);
r_selected = r(start_idx:end_idx);
u2_selected = u1(start_idx:end_idx);
u1_selected = u2(start_idx:end_idx);

pwm1_selected = arrayfun(@(x) convertThrustToPwm(x), u1_selected);
pwm2_selected = arrayfun(@(x) convertSteeringToPwm(x), u2_selected);

% YALMIP 변수를 정의합니다.
Yv = sdpvar(1,1); Yvv = sdpvar(1,1);
Yn = sdpvar(1,1); Nr = sdpvar(1,1);
Nrr = sdpvar(1,1); Nv = sdpvar(1,1);
b3 = sdpvar(1,1);

% 초기 추정값 설정
initial_guess = [0; 0; 0; 0; 0; 0; 0]; % Xu, Xuu, Yv, Yvv, Yn, Nr, Nrr, Nv, b1, b2

% 최적화 문제 설정
objective = @(params) computeObjective(params, v_selected, r_selected, pwm1_selected, pwm2_selected, dt);

% 제약 조건 설정: Xu > 0, Xuu > 0
nonlcon = @(params) constraints(params);  % 비선형 제약 함수

% 최적화 옵션
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'interior-point');

% fmincon을 사용하여 최적화 수행
[opt_params, fval] = fmincon(objective, initial_guess, [], [], [], [], [], [], nonlcon, options);

% 최적화된 파라미터 추출
Yv_opt = opt_params(1);
Yvv_opt = opt_params(2);
Yn_opt = opt_params(3);
Nr_opt = opt_params(4);
Nrr_opt = opt_params(5);
Nv_opt = opt_params(6);
b3_opt = opt_params(7);

disp('Optimized Parameters:');
disp(['Yv: ', num2str(Yv_opt)]);
disp(['Yvv: ', num2str(Yvv_opt)]);
disp(['Yn: ', num2str(Yn_opt)]);
disp(['Nr: ', num2str(Nr_opt)]);
disp(['Nrr: ', num2str(Nrr_opt)]);
disp(['Nv: ', num2str(Nv_opt)]);
disp(['b3: ', num2str(b3_opt)]);

% 예측된 u, v, r 값 계산 (모델에 의해 예측된 값)
u_predicted = zeros(1, length(t));
v_predicted = zeros(1, length(t));
r_predicted = zeros(1, length(t));

u_predicted(1) = u_selected(1); % 초기값 설정
v_predicted(1) = v_selected(1);
r_predicted(1) = r_selected(1);

for i = 2:length(t)
    % 시스템 동역학을 기반으로 다음 상태를 예측 (최적화된 계수 사용)
    v_predicted(i) = (-Yv_opt*v_predicted(i-1) - Yvv_opt*abs(v_predicted(i-1))*v_predicted(i-1) - Yn_opt*r_predicted(i-1) + b1*pwm1_selected(i)*pwm1_selected(i-1)*sin(b2*pwm2_selected(i-1)))*dt+ v_predicted(i-1);
    r_predicted(i) = (-Nr_opt*r_predicted(i-1) - Nrr_opt*abs(r_predicted(i-1))*r_predicted(i-1) - Nv_opt*v_predicted(i-1) - b3_opt*b1*pwm1_selected(i-1)*pwm1_selected(i-1)*sin(b2*pwm2_selected(i-1)))*dt + r_predicted(i-1);
    % v_predicted(i) = (-Yv_opt*v_predicted(i-1) - Yvv_opt*abs(v_predicted(i-1))*v_predicted(i-1) - Yn_opt*r_predicted(i-1) + 0.00073077*pwm1_selected(i)*pwm1_selected(i-1)*sin(b2_opt*pwm2_selected(i-1)))*dt+ v_predicted(i-1);
    % r_predicted(i) = (-Nr_opt*r_predicted(i-1) - Nrr_opt*abs(r_predicted(i-1))*r_predicted(i-1) - Nv_opt*v_predicted(i-1) - 4.0*0.00073077*pwm1_selected(i-1)*pwm1_selected(i-1)*sin(b2_opt*pwm2_selected(i-1)))*dt + r_predicted(i-1);
end

% 실제 값과 예측 값을 비교하는 그래프 그리기
figure;


% v 값 비교
subplot(4,1,1);
plot(t, v_selected, 'b', 'DisplayName', 'v actual'); hold on;
plot(t, v_predicted, 'r--', 'DisplayName', 'v predicted');
xlabel('Time [s]');
ylabel('v');
legend;
title('Comparison of v actual and v predicted');

% r 값 비교
subplot(4,1,2);
plot(t, r_selected, 'b', 'DisplayName', 'r actual'); hold on;
plot(t, r_predicted, 'r--', 'DisplayName', 'r predicted');
xlabel('Time [s]');
ylabel('r');
legend;
title('Comparison of r actual and r predicted');

% u1 값 비교
subplot(4,1,3);
plot(t, pwm1_selected, 'b', 'DisplayName', 'u1 actual'); hold on;
xlabel('Time [s]');
ylabel('u1');
legend;
title('Comparison of u1 actual and u1 predicted');

% u2 값 비교
subplot(4,1,4);
plot(t, pwm2_selected, 'b', 'DisplayName', 'u2 actual'); hold on;
xlabel('Time [s]');
ylabel('u2');
legend;
title('Comparison of u2 actual and u2 predicted');

% 목적 함수 정의 (최소화하려는 오차 함수)
function objective = computeObjective(params, v_selected, r_selected, pwm1_selected, pwm2_selected, dt)
    Yv = params(1);
    Yvv = params(2);
    Yn = params(3);
    Nr = params(4);
    Nrr = params(5);
    Nv = params(6);
    b2 = 0.0040635;
    b3 = params(7);
    b1 = 0.00058466;

    objective = 0;
    v_next = v_selected(1);
    r_next = r_selected(1);
    
    for i = 1:length(v_selected)-1
        % 시스템 동역학을 기반으로 다음 상태를 예측
        v_next = (-Yv*v_next - Yvv*abs(v_next).*v_next - Yn*r_next + b1*pwm1_selected(i)*pwm1_selected(i)*sin(b2*pwm2_selected(i)))*dt + v_next;
        r_next = (-Nr*r_next - Nrr*abs(r_next).*r_next - Nv*v_next - b3*b1*pwm1_selected(i)*pwm1_selected(i)*sin(b2*pwm2_selected(i)))*dt + r_next;
        
        % v_next = (-Yv*v_next - Yvv*abs(v_next).*v_next - Yn*r_next + 0.00073077*pwm1_selected(i)*pwm1_selected(i)*sin(b2*pwm2_selected(i)))*dt + v_next;
        % r_next = (-Nr*r_next - Nrr*abs(r_next).*r_next - Nv*v_next - 4.0*0.00073077*pwm1_selected(i)*pwm1_selected(i)*sin(b2*pwm2_selected(i)))*dt + r_next;
         
        % 실제 변화량과 예측값의 차이를 최소화하는 오차 함수
        v_diff = (v_next - v_selected(i+1))^2;
        r_diff = (r_next - r_selected(i+1))^2;
        
        % 오차 함수 추가
        objective = objective + 4*v_diff + 100*r_diff;
    end
end

% 제약 조건 함수 (Xu > 0, Xuu > 0)
function [c, ceq] = constraints(params)
    Yv = params(1);
    Yvv = params(2);
    Nr = params(3);
    Nrr = params(4);
    Nv = params(5);
    Yr = params(6);
    b3 = params(7);
    % Xu > 0, Xuu > 0 의 조건
    c = [-Yv; -Yvv;-Nr; -Nrr;-Nv; -Yr; -b3];  % 부등식 제약: Xu > 0, Xuu > 0
    ceq = [];  % 평등 제약은 없음
end


