clc; clear; close all;

% 모델 생성
model.obj = [1, 2, 3, 4];  % 목적 함수 계수 (선형 부분)
model.Q = sparse([1, 0, 0, 0;
                  0, 1, 0, 0;
                  0, 0, 1, 0;
                  0, 0, 0, 1]);  % 이차 목적 함수 부분
model.modelsense = 'min';  % 최소화 문제

% 제약 조건 정의
model.A = sparse([        
    2, 0, 1, 0;
    0, 3, 0, 1;        
]);

model.rhs = [-5; -6];
model.sense = ['<'; '<'];

% 변수의 하한과 상한 설정 (음수 값을 허용)
model.lb = [-10; -5; -8; -3];  % 변수 각각 음수 값이 허용
model.ub = [10; 10; 10; 10];  % 상한 설정

% 변수의 타입 설정 (연속형 변수)
model.vtype = 'CCCC';  % 연속형 변수 (C는 continuous)

% 모델 최적화 실행
result = gurobi(model, struct('OutputFlag', 1));

% 결과 출력
disp(['x1 (실수 변수): ', num2str(result.x(1))]);
disp(['x2 (실수 변수): ', num2str(result.x(2))]);
disp(['x3 (실수 변수): ', num2str(result.x(3))]);
disp(['x4 (실수 변수): ', num2str(result.x(4))]);
