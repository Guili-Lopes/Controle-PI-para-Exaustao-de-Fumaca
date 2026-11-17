%% PROJETO DE EXAUSTAO - PLANTA E CONTROLADOR PI
% Simulacao academica simplificada.
% Planta em torno de um ponto de operacao de 50% de PWM.
% O modelo nao representa medidas reais do MQ-2 ou da ventoinha.
% Requer Control System Toolbox.

clear; clc; close all;

%% Parametros do projeto
s = tf('s');
G = (2.5*s - 1)/(100*s^3 + 95*s^2 + 24.5*s + 1);

Kp = -1.5;
Ki = -0.0675;
Ts = 0.1;                 % PI executado a cada 100 ms
PWM_base = 0.50;
PWM_min = 0;
PWM_max = 1;
I_min = -0.50;
I_max = 0.50;

t = 0:Ts:160;

%% 1. Planta em malha aberta
% Incremento de 30 pontos percentuais em relacao a 50% de PWM.
% Assim, para a planta linearizada, a entrada e Delta_PWM = +0.30.
delta_pwm = 0.30*(t >= 5);
saida_aberta = lsim(G, delta_pwm, t);

fig1 = figure('Color','w','Name','Planta em malha aberta');
plot(t, delta_pwm, '--', 'LineWidth', 1.5); hold on;
plot(t, saida_aberta, 'LineWidth', 1.5);
grid on; xlabel('Tempo (s)');
ylabel('Variacao normalizada');
title('Planta G(s) em malha aberta');
legend('Incremento de PWM','Saida da planta','Location','best');

%% 2. Controlador PI discreto em malha fechada
% Referencia negativa: representa reducao relativa da grandeza controlada.
% Como G(0)=-1, aumentar o PWM tende a reduzir a saida da planta.
referencia = -0.30*(t >= 5);
Gd = c2d(ss(G), Ts, 'zoh');
[A, B, C, D] = ssdata(Gd);

x = zeros(size(A,1),1);
integral = 0;
saida = zeros(size(t));
pwm = zeros(size(t));

for k = 1:numel(t)
    y = C*x;                    % D = 0 para esta planta
    erro = referencia(k) - y;

    % Euler regressivo, como no firmware
    integral = integral + Ki*Ts*erro;
    integral = min(I_max, max(I_min, integral)); % clamping

    proporcional = Kp*erro;
    comando = PWM_base + proporcional + integral;
    comando = min(PWM_max, max(PWM_min, comando));

    saida(k) = y;
    pwm(k) = comando;

    % Planta recebe o desvio em relacao ao PWM nominal de 50%.
    x = A*x + B*(comando-PWM_base);
end

fig2 = figure('Color','w','Name','PI em malha fechada');
subplot(2,1,1);
plot(t, referencia, 'k--', 'LineWidth', 1.5); hold on;
plot(t, saida, 'LineWidth', 1.5);
grid on; ylabel('Saida relativa');
title('Planta com PI discreto (Euler regressivo)');
legend('Referencia','Saida da planta','Location','best');
subplot(2,1,2);
plot(t, 100*pwm, 'LineWidth', 1.5);
grid on; xlabel('Tempo (s)'); ylabel('PWM (%)');
ylim([0 105]); title('Acionamento do ventilador');

%% Salvar figuras para o relatorio
pasta = 'resultados_simplificados';
if ~exist(pasta,'dir'), mkdir(pasta); end
exportgraphics(fig1,fullfile(pasta,'01_planta_malha_aberta.png'),'Resolution',200);
exportgraphics(fig2,fullfile(pasta,'02_planta_com_PI.png'),'Resolution',200);

fprintf('G(s) = (2.5s - 1)/(100s^3 + 95s^2 + 24.5s + 1)\n');
fprintf('PI: Kp=%.4f, Ki=%.4f, Ts=%.2f s\n', Kp,Ki,Ts);
fprintf('PWM final: %.1f%% | Saida final: %.4f\n', 100*pwm(end), saida(end));
fprintf('Figuras salvas em: %s\n', pasta);
