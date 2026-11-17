%% CRIAR MODELO SIMULINK SIMPLIFICADO - PLANTA E PI
% Gera um unico modelo com planta aberta e planta com PI em malha fechada.
% PI implementado com Euler regressivo, clamping e PWM de 0 a 100%.
% Requer Simulink e Control System Toolbox.

clear; clc;

s = tf('s');
G = (2.5*s - 1)/(100*s^3 + 95*s^2 + 24.5*s + 1);
Ts = 0.1;
Kp = -1.5;
Ki = -0.0675;
base = 0.50;
[A,B,C,D] = ssdata(c2d(ss(G), Ts, 'zoh'));

mdl = 'Planta_e_PI_Exaustao';
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl);
set_param(mdl, 'Solver','FixedStepDiscrete', 'FixedStep',num2str(Ts,17), 'StopTime','160');

%% Planta em malha aberta: incremento de PWM em +0.30
add_block('simulink/Sources/Step',[mdl '/Degrau_PWM'], ...
    'Time','5','Before','0','After','0.30','SampleTime',num2str(Ts), ...
    'Position',[35 75 65 105]);
add_planta(mdl,'Planta_Aberta', [120 65 260 115], A,B,C,D,Ts);
add_block('simulink/Sinks/Scope',[mdl '/Scope_Planta'], ...
    'Position',[320 75 370 105]);
ligar(mdl,'Degrau_PWM/1','Planta_Aberta/1');
ligar(mdl,'Planta_Aberta/1','Scope_Planta/1');

%% Planta em malha fechada com controlador PI
add_block('simulink/Sources/Step',[mdl '/Referencia'], ...
    'Time','5','Before','0','After','-0.30','SampleTime',num2str(Ts), ...
    'Position',[35 310 65 340]);
add_block('simulink/Math Operations/Sum',[mdl '/Erro'], ...
    'Inputs','+-','Position',[115 305 145 345]);
ligar(mdl,'Referencia/1','Erro/1');

add_block('simulink/Math Operations/Gain',[mdl '/Kp'], ...
    'Gain',num2str(Kp,17),'Position',[190 245 255 275]);
ligar(mdl,'Erro/1','Kp/1');
add_block('simulink/Math Operations/Gain',[mdl '/Ki_Ts'], ...
    'Gain',num2str(Ki*Ts,17),'Position',[190 365 255 395]);
ligar(mdl,'Erro/1','Ki_Ts/1');
add_block('simulink/Discrete/Unit Delay',[mdl '/Integral_anterior'], ...
    'InitialCondition','0','SampleTime',num2str(Ts), ...
    'Position',[315 430 365 470]);
add_block('simulink/Math Operations/Sum',[mdl '/Atualiza_Integral'], ...
    'Inputs','++','Position',[315 355 345 395]);
ligar(mdl,'Ki_Ts/1','Atualiza_Integral/1');
ligar(mdl,'Integral_anterior/1','Atualiza_Integral/2');
add_block('simulink/Discontinuities/Saturation',[mdl '/Clamping'], ...
    'LowerLimit','-0.5','UpperLimit','0.5', ...
    'Position',[395 355 445 395]);
ligar(mdl,'Atualiza_Integral/1','Clamping/1');
ligar(mdl,'Clamping/1','Integral_anterior/1');

add_block('simulink/Sources/Constant',[mdl '/PWM_base_50pct'], ...
    'Value',num2str(base),'Position',[385 190 420 220]);
add_block('simulink/Math Operations/Sum',[mdl '/Comando_PI'], ...
    'Inputs','+++','Position',[490 270 520 340]);
ligar(mdl,'PWM_base_50pct/1','Comando_PI/1');
ligar(mdl,'Kp/1','Comando_PI/2');
ligar(mdl,'Clamping/1','Comando_PI/3');
add_block('simulink/Discontinuities/Saturation',[mdl '/PWM_0_a_100'], ...
    'LowerLimit','0','UpperLimit','1', ...
    'Position',[565 285 625 325]);
ligar(mdl,'Comando_PI/1','PWM_0_a_100/1');

add_block('simulink/Math Operations/Sum',[mdl '/Desvio_PWM'], ...
    'Inputs','+-','Position',[690 295 720 335]);
ligar(mdl,'PWM_0_a_100/1','Desvio_PWM/1');
ligar(mdl,'PWM_base_50pct/1','Desvio_PWM/2');
add_planta(mdl,'Planta_com_PI',[775 285 905 335],A,B,C,D,Ts);
ligar(mdl,'Desvio_PWM/1','Planta_com_PI/1');
ligar(mdl,'Planta_com_PI/1','Erro/2');

% Comparacao entre referencia e saida no mesmo Scope
add_block('simulink/Signal Routing/Mux',[mdl '/Mux_Referencia_Saida'], ...
    'Inputs','2','Position',[960 260 965 340]);
ligar(mdl,'Referencia/1','Mux_Referencia_Saida/1');
ligar(mdl,'Planta_com_PI/1','Mux_Referencia_Saida/2');
add_block('simulink/Sinks/Scope',[mdl '/Scope_PI'], ...
    'Position',[1020 280 1070 315]);
ligar(mdl,'Mux_Referencia_Saida/1','Scope_PI/1');
add_block('simulink/Sinks/Scope',[mdl '/Scope_PWM'], ...
    'Position',[690 385 740 420]);
ligar(mdl,'PWM_0_a_100/1','Scope_PWM/1');

save_system(mdl,[mdl '.slx']);
open_system(mdl);
fprintf('Modelo gerado: %s.slx\n',mdl);
fprintf('Execute Run e abra Scope_Planta, Scope_PI e Scope_PWM.\n');

%% Funcoes auxiliares
function add_planta(mdl,nome,position,A,B,C,D,Ts)
    add_block('simulink/Discrete/Discrete State-Space',[mdl '/' nome], ...
        'A',mat2str(A,17),'B',mat2str(B,17), ...
        'C',mat2str(C,17),'D',mat2str(D,17), ...
        'InitialCondition','0','SampleTime',num2str(Ts,17), ...
        'Position',position);
end

function ligar(mdl,origem,destino)
    add_line(mdl,origem,destino,'autorouting','on');
end
