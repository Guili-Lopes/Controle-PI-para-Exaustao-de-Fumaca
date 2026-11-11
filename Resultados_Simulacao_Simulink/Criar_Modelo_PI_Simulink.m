%% GERAR MODELO SIMULINK - COMPARACAO DE CONTROLADORES PI
% Projeto academico: exaustao, planta linear nominal
%
% G(s) = (2.5*s - 1)/(100*s^3 + 95*s^2 + 24.5*s + 1)
% Kp=-1.5 | Ki=-0.0675 | Ts=0.1s | Kaw=0.045
%
% Quatro modelos em malha fechada:
%   1) Ideal (sem saturacao)
%   2) PWM limitado, sem anti-windup
%   3) Back-Calculation
%   4) Clamping direto do integrador
%
% IMPORTANTE: todas as quatro malhas usam Euler regressivo:
%    I[k] = I[k-1] + Ki*Ts*e[k].
% A planta recebe o INCREMENTO de duty em torno de 50% nominal.
% O 50% e uma hipotese da simulacao, nao uma medicao da ventoinha.
%
% Requer: MATLAB + Simulink + Control System Toolbox.
% Execute no MATLAB. O script cria, salva e abre o arquivo .slx.

clear; clc;

%% 1. Parametros e discretizacao ZOH da planta
s = tf('s');
G = (2.5*s - 1)/(100*s^3 + 95*s^2 + 24.5*s + 1);
Ts = 0.1;
Kp = -1.5;
Ki = -0.0675;
Kaw = abs(Ki/Kp); % 0.045; Tt = abs(Kp/Ki)
pwm_base = 0.50;
pwm_min = 0;
pwm_max = 1;
I_min = -0.50;     % clamping direto: hipotese inicial
I_max =  0.50;

Gd = c2d(ss(G),Ts,'zoh');
[A,B,C,D] = ssdata(Gd);

%% 2. Modelo novo
mdl = 'comparacao_PI_Simulink';
if bdIsLoaded(mdl)
    close_system(mdl,0);
end
new_system(mdl);
set_param(mdl, ...
    'Solver','FixedStepDiscrete', ...
    'FixedStep',num2str(Ts,17), ...
    'StopTime','240', ...
    'ReturnWorkspaceOutputs','on');

% Fonte da referencia relativa:
% t=5:   -0.30
% t=80:  -0.65  (acrescenta -0.35)
% t=150: -0.30  (acrescenta +0.35)
add_block('simulink/Sources/Step',[mdl '/Degrau_5s'], ...
    'Position',[40 45 110 75],'Time','5','Before','0', ...
    'After','-0.30','SampleTime',num2str(Ts));
add_block('simulink/Sources/Step',[mdl '/Degrau_80s'], ...
    'Position',[40 95 110 125],'Time','80','Before','0', ...
    'After','-0.35','SampleTime',num2str(Ts));
add_block('simulink/Sources/Step',[mdl '/Degrau_150s'], ...
    'Position',[40 145 110 175],'Time','150','Before','0', ...
    'After','0.35','SampleTime',num2str(Ts));
add_block('simulink/Math Operations/Sum',[mdl '/Referencia'], ...
    'Position',[155 82 190 145],'Inputs','+++');
ligar(mdl,'Degrau_5s/1','Referencia/1');
ligar(mdl,'Degrau_80s/1','Referencia/2');
ligar(mdl,'Degrau_150s/1','Referencia/3');

add_block('simulink/Sinks/To Workspace',[mdl '/Salvar_Referencia'], ...
    'Position',[230 75 345 105], ...
    'VariableName','referencia_PI','SaveFormat','Timeseries');
ligar(mdl,'Referencia/1','Salvar_Referencia/1');

%% 3. Quatro controladores em malha fechada
modos = {'ideal','saturado','back_calculation','clamping'};
nomes = {'PI_Ideal','PI_Saturado','PI_BackCalculation','PI_Clamping'};
ypos  = [235 415 595 775];

for n = 1:numel(modos)
    sub = [mdl '/' nomes{n}];
    add_block('simulink/Ports & Subsystems/Subsystem',sub, ...
        'Position',[330 ypos(n) 535 ypos(n)+105]);
    % Remove a estrutura In1 -> Out1 fornecida por padrao.
    Simulink.SubSystem.deleteContents(sub);
    construir_malha(sub,modos{n},Ts,Kp,Ki,Kaw, ...
        pwm_base,pwm_min,pwm_max,I_min,I_max,A,B,C,D);

    ligar(mdl,'Referencia/1',[nomes{n} '/1']);
    sufixo = modos{n};
    % Saida da planta (y)
    add_block('simulink/Sinks/To Workspace',[mdl '/Salvar_y_' sufixo], ...
        'Position',[635 ypos(n)-2 760 ypos(n)+23], ...
        'VariableName',['y_' sufixo],'SaveFormat','Timeseries');
    ligar(mdl,[nomes{n} '/1'],['Salvar_y_' sufixo '/1']);

    % Duty aplicado (0..1, exceto caso ideal)
    add_block('simulink/Sinks/To Workspace',[mdl '/Salvar_pwm_' sufixo], ...
        'Position',[635 ypos(n)+38 760 ypos(n)+63], ...
        'VariableName',['pwm_' sufixo],'SaveFormat','Timeseries');
    ligar(mdl,[nomes{n} '/2'],['Salvar_pwm_' sufixo '/1']);

    % Integral acumulada
    add_block('simulink/Sinks/To Workspace',[mdl '/Salvar_I_' sufixo], ...
        'Position',[635 ypos(n)+78 760 ypos(n)+103], ...
        'VariableName',['I_' sufixo],'SaveFormat','Timeseries');
    ligar(mdl,[nomes{n} '/3'],['Salvar_I_' sufixo '/1']);
end

%% 4. Scopes para visualizar resultados diretamente no Simulink
add_block('simulink/Signal Routing/Mux',[mdl '/Mux_Saidas'], ...
    'Inputs','5','Position',[880 300 885 440]);
ligar(mdl,'Referencia/1','Mux_Saidas/1');
for n = 1:4
    ligar(mdl,[nomes{n} '/1'],sprintf('Mux_Saidas/%d',n+1));
end
add_block('simulink/Sinks/Scope',[mdl '/Scope_Saidas'], ...
    'Position',[950 350 1035 390]);
ligar(mdl,'Mux_Saidas/1','Scope_Saidas/1');

add_block('simulink/Signal Routing/Mux',[mdl '/Mux_PWM'], ...
    'Inputs','4','Position',[880 565 885 695]);
for n = 1:4
    ligar(mdl,[nomes{n} '/2'],sprintf('Mux_PWM/%d',n));
end
add_block('simulink/Sinks/Scope',[mdl '/Scope_PWM'], ...
    'Position',[950 605 1035 645]);
ligar(mdl,'Mux_PWM/1','Scope_PWM/1');

add_block('simulink/Signal Routing/Mux',[mdl '/Mux_Integral'], ...
    'Inputs','4','Position',[880 775 885 905]);
for n = 1:4
    ligar(mdl,[nomes{n} '/3'],sprintf('Mux_Integral/%d',n));
end
add_block('simulink/Sinks/Scope',[mdl '/Scope_Integral'], ...
    'Position',[950 815 1035 855]);
ligar(mdl,'Mux_Integral/1','Scope_Integral/1');

%% 5. Salvar e abrir
save_system(mdl,[mdl '.slx']);
open_system(mdl);
fprintf('\nModelo criado: %s.slx\n',mdl);
fprintf('Clique em Run no Simulink; abra Scope_Saidas, Scope_PWM e Scope_Integral.\n');
fprintf('Para ler dados em MATLAB: out = sim(''%s'');\n',mdl);
fprintf('Exemplo: out.y_back_calculation.Time / out.y_back_calculation.Data\n');

%% ==============================================================
% FUNCAO: construcao interna de uma malha
% ===============================================================
function construir_malha(sub,modo,Ts,Kp,Ki,Kaw,pwm_base,umin,umax,Imin,Imax,A,B,C,D)
    add_block('simulink/Ports & Subsystems/In1',[sub '/Referencia'], ...
        'Position',[25 70 55 90],'Port','1');
    add_block('simulink/Ports & Subsystems/Out1',[sub '/Saida_Planta'], ...
        'Position',[900 95 930 115],'Port','1');
    add_block('simulink/Ports & Subsystems/Out1',[sub '/Duty_PWM'], ...
        'Position',[900 210 930 230],'Port','2');
    add_block('simulink/Ports & Subsystems/Out1',[sub '/Termo_Integral'], ...
        'Position',[900 330 930 350],'Port','3');

    % erro = referencia - planta
    add_block('simulink/Math Operations/Sum',[sub '/Erro'], ...
        'Position',[105 92 135 145],'Inputs','+-');
    ligar(sub,'Referencia/1','Erro/1');

    % P[k] = Kp*e[k]
    add_block('simulink/Math Operations/Gain',[sub '/Ganho_Kp'], ...
        'Position',[185 82 255 122],'Gain',num2str(Kp,17));
    ligar(sub,'Erro/1','Ganho_Kp/1');

    % Delta I[k] = Ki*Ts*e[k] (Euler regressivo)
    add_block('simulink/Math Operations/Gain',[sub '/Ganho_KiTs'], ...
        'Position',[185 160 260 200],'Gain',num2str(Ki*Ts,17));
    ligar(sub,'Erro/1','Ganho_KiTs/1');
    add_block('simulink/Discrete/Unit Delay',[sub '/Integral_anterior'], ...
        'Position',[160 265 215 305], ...
        'InitialCondition','0','SampleTime',num2str(Ts,17));
    add_block('simulink/Math Operations/Sum',[sub '/Integral_Euler'], ...
        'Position',[310 185 340 225],'Inputs','++');
    ligar(sub,'Integral_anterior/1','Integral_Euler/1');
    ligar(sub,'Ganho_KiTs/1','Integral_Euler/2');

    % Clamping direto do estado integral: Imin <= I <= Imax.
    % O valor limitado REALIMENTA o Unit Delay.
    valor_integral_pedido = 'Integral_Euler/1';
    valor_integral_proximo = 'Integral_Euler/1';
    if strcmp(modo,'clamping')
        add_block('simulink/Discontinuities/Saturation',[sub '/Limite_Integral'], ...
            'Position',[370 185 415 225], ...
            'UpperLimit',num2str(Imax,17), ...
            'LowerLimit',num2str(Imin,17));
        ligar(sub,'Integral_Euler/1','Limite_Integral/1');
        valor_integral_pedido = 'Limite_Integral/1';
        valor_integral_proximo = 'Limite_Integral/1';
    end

    % u_req = pwm_base + P[k] + I[k]
    add_block('simulink/Sources/Constant',[sub '/PWM_Nominal'], ...
        'Position',[420 42 475 75],'Value',num2str(pwm_base,17));
    add_block('simulink/Math Operations/Sum',[sub '/Duty_Solicitado'], ...
        'Position',[510 90 545 170],'Inputs','+++');
    ligar(sub,'PWM_Nominal/1','Duty_Solicitado/1');
    ligar(sub,'Ganho_Kp/1','Duty_Solicitado/2');
    ligar(sub,valor_integral_pedido,'Duty_Solicitado/3');

    % Sem saturacao no PI ideal. Nos outros, limita PWM [0,1].
    duty_aplicado = 'Duty_Solicitado/1';
    if ~strcmp(modo,'ideal')
        add_block('simulink/Discontinuities/Saturation',[sub '/Limite_PWM'], ...
            'Position',[585 105 635 155], ...
            'UpperLimit',num2str(umax,17), ...
            'LowerLimit',num2str(umin,17));
        ligar(sub,'Duty_Solicitado/1','Limite_PWM/1');
        duty_aplicado = 'Limite_PWM/1';
    end

    if strcmp(modo,'back_calculation')
        % BackCalculation:
        % I_corrigido = I_Euler + Kaw*Ts*(u_sat - u_req).
        add_block('simulink/Math Operations/Sum',[sub '/Erro_Saturacao'], ...
            'Position',[665 245 695 290],'Inputs','+-');
        ligar(sub,'Limite_PWM/1','Erro_Saturacao/1');
        ligar(sub,'Duty_Solicitado/1','Erro_Saturacao/2');
        add_block('simulink/Math Operations/Gain',[sub '/Ganho_KawTs'], ...
            'Position',[730 245 810 290],'Gain',num2str(Kaw*Ts,17));
        ligar(sub,'Erro_Saturacao/1','Ganho_KawTs/1');
        add_block('simulink/Math Operations/Sum',[sub '/Integral_Corrigida'], ...
            'Position',[775 315 805 355],'Inputs','++');
        ligar(sub,'Integral_Euler/1','Integral_Corrigida/1');
        ligar(sub,'Ganho_KawTs/1','Integral_Corrigida/2');
        valor_integral_proximo = 'Integral_Corrigida/1';
    end

    ligar(sub,valor_integral_proximo,'Integral_anterior/1');
    ligar(sub,valor_integral_proximo,'Termo_Integral/1');
    ligar(sub,duty_aplicado,'Duty_PWM/1');

    % Entrada da planta: incremento em torno do PWM nominal 50%.
    add_block('simulink/Math Operations/Sum',[sub '/Desvio_PWM'], ...
        'Position',[675 85 705 145],'Inputs','+-');
    ligar(sub,duty_aplicado,'Desvio_PWM/1');
    ligar(sub,'PWM_Nominal/1','Desvio_PWM/2');

    % Gd(z) = ZOH{G(s)}, forma de estados discretos.
    add_block('simulink/Discrete/Discrete State-Space',[sub '/Planta_ZOH'], ...
        'Position',[745 90 855 140], ...
        'A',mat2str(A,17), 'B',mat2str(B,17), ...
        'C',mat2str(C,17), 'D',mat2str(D,17), ...
        'InitialCondition','0','SampleTime',num2str(Ts,17));
    ligar(sub,'Desvio_PWM/1','Planta_ZOH/1');
    ligar(sub,'Planta_ZOH/1','Saida_Planta/1');
    ligar(sub,'Planta_ZOH/1','Erro/2');
end

%% ==============================================================
% FUNCAO: conectar blocos com roteamento automatico
% ===============================================================
function ligar(modelo,origem,destino)
    add_line(modelo,origem,destino,'autorouting','on');
end
