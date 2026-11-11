%% SISTEMA DE EXAUSTAO - PI DISCRETO E ANTI-WINDUP
% Compara PI ideal, PI com saturacao, PI com Back-Calculation e PI com
% clamping direto do integrador. As quatro simulacoes principais usam
% o MESMO integrador Euler regressivo, escolhido para o PIC18F4550.
% Inclui Back-Calculation + Tustin para conferir o estudo anterior.
%
% Requisito: MATLAB com Control System Toolbox.
% Saidas: graficos PNG, tabelas CSV e dados MAT em 'resultados_PI'.
% Modelo academico: nao representa calibracao real do MQ-2 ou da ventoinha.

clear; clc; close all;

%% 1) Planta e parametros
s = tf('s');
G = (2.5*s - 1)/(100*s^3 + 95*s^2 + 24.5*s + 1);

Kp = -1.5;
Ki = -0.0675;
Ts = 0.1;                    % 100 ms = 10 Hz (periodo do PI)
Kaw = abs(Ki/Kp);           % 0.045 1/s: tempo de rastreamento Tt=Ti

pwm_base = 0.50;            % ponto de operacao HIPOTETICO de 50%
pwm_min = 0.0;
pwm_max = 1.0;

% CLAMPING DIRETO: limites fixos da parcela integral.
% Com erro zero (P=0), essa faixa permite de 0% a 100% de PWM.
% Sao limites de projeto escolhidos para este estudo, NAO sao Kaw.
I_min = pwm_min - pwm_base;  % -0.50
I_max = pwm_max - pwm_base;  % +0.50

fprintf('PI: Kp = %.4f | Ki = %.4f | Ts = %.3f s\n', Kp, Ki, Ts);
fprintf('Back-Calculation: Kaw = |Ki/Kp| = %.4f 1/s\n', Kaw);
fprintf('Clamping direto: Imin = %.2f | Imax = %.2f\n\n', I_min, I_max);

%% 2) Discretizacao ZOH da planta
% O controle e digital (100 ms); a planta recebe o PWM fixo entre amostras.
Gd = c2d(ss(G), Ts, 'zoh');
[A, B, C, D] = ssdata(Gd);
assert(all(abs(D(:)) < 1e-12), 'A planta deveria ser estritamente propria.');

% Periodo total e sequencia de referencias RELATIVAS ao ponto nominal.
t = 0:Ts:240;
r = zeros(size(t));
r(t >= 5   & t < 80)  = -0.30;
r(t >= 80  & t < 150) = -0.65; % inviavel: pede ~115% de PWM
r(t >= 150)          = -0.30;

%% 3) Simular os controladores
p = struct('Kp',Kp,'Ki',Ki,'Ts',Ts,'Kaw',Kaw, ...
           'base',pwm_base,'umin',pwm_min,'umax',pwm_max, ...
           'Imin',I_min,'Imax',I_max);

ideal  = simular_pi(A,B,C,t,r,p,'ideal');
sat    = simular_pi(A,B,C,t,r,p,'saturado');
back   = simular_pi(A,B,C,t,r,p,'back_euler');
clamp  = simular_pi(A,B,C,t,r,p,'clamping');
back_T = simular_pi(A,B,C,t,r,p,'back_tustin');

%% 4) Metricas da primeira resposta e da recuperacao
nomes = {'Ideal - Euler'; 'Saturado - Euler'; ...
         'Back-Calc - Euler'; 'Clamping - Euler'; ...
         'Back-Calc - Tustin antigo'};
casos = {ideal; sat; back; clamp; back_T};
N = numel(casos);

subida = zeros(N,1); acomodacao = zeros(N,1); sobressinal = zeros(N,1);
iae_recu = zeros(N,1); y_final = zeros(N,1); pwm_final = zeros(N,1);

for j = 1:N
    m = metricas_pi(t,r,casos{j});
    subida(j) = m.subida;
    acomodacao(j) = m.acomodacao;
    sobressinal(j) = m.sobressinal;
    iae_recu(j) = m.iae;
    y_final(j) = m.yfinal;
    pwm_final(j) = m.pwmfinal;
end

resumo = table(nomes,subida,acomodacao,sobressinal,iae_recu,y_final,pwm_final, ...
    'VariableNames',{'Caso','Subida_s','Acomodacao_s','Sobressinal_pct', ...
                     'IAE_150_240','Saida_em_240','PWM_final_pct'});
disp(resumo);

reducao_back = 100*(1 - iae_recu(3)/iae_recu(2));
reducao_clamp = 100*(1 - iae_recu(4)/iae_recu(2));
fprintf('\nReducao IAE vs PI saturado: Back-Calculation Euler = %.2f%%\n',reducao_back);
fprintf('Reducao IAE vs PI saturado: Clamping Euler = %.2f%%\n',reducao_clamp);

%% 5) Graficos principais (somente Euler para comparacao justa)
legendas = {'Referencia','Ideal','Saturado sem AW','Back-Calculation','Clamping'};

fig1 = figure('Name','Comparacao PI discreto - Euler','Color','w');
subplot(3,1,1);
plot(t,r,'k--','LineWidth',1.5); hold on;
plot(t,ideal.y,t,sat.y,t,back.y,t,clamp.y,'LineWidth',1.2);
grid on; ylabel('Saida relativa');
title('Resposta da planta - PI com Euler regressivo');
legend(legendas,'Location','best');

subplot(3,1,2);
plot(t,100*ideal.duty,t,100*sat.duty,t,100*back.duty, ...
     t,100*clamp.duty,'LineWidth',1.2); hold on;
yline(0,'k:'); yline(100,'k:');
grid on; ylabel('PWM (%)');
title('Atuacao solicitada / aplicada (ideal pode sair de 0-100%)');
legend(legendas(2:end),'Location','best');

subplot(3,1,3);
plot(t,ideal.I,t,sat.I,t,back.I,t,clamp.I,'LineWidth',1.2); hold on;
yline(I_min,'k:'); yline(I_max,'k:');
grid on; xlabel('Tempo (s)'); ylabel('Integral');
title('Estado do integrador e limites do clamping');
legend(legendas(2:end),'Location','best');

fig2 = figure('Name','Recuperacao apos saturacao','Color','w');
selecao = (t >= 140 & t <= 240);
subplot(2,1,1);
plot(t(selecao),r(selecao),'k--','LineWidth',1.5); hold on;
plot(t(selecao),sat.y(selecao),t(selecao),back.y(selecao), ...
     t(selecao),clamp.y(selecao),'LineWidth',1.3);
grid on; ylabel('Saida relativa');
title('Recuperacao apos referencia inviavel (retorno em t=150 s)');
legend({'Referencia','Saturado sem AW','Back-Calculation','Clamping'},'Location','best');

subplot(2,1,2);
plot(t(selecao),100*sat.duty(selecao),t(selecao),100*back.duty(selecao), ...
     t(selecao),100*clamp.duty(selecao),'LineWidth',1.3); hold on;
yline(100,'k:');
grid on; ylabel('PWM (%)'); xlabel('Tempo (s)');
legend({'Saturado sem AW','Back-Calculation','Clamping'},'Location','best');

fig3 = figure('Name','Tustin anterior vs Euler novo','Color','w');
subplot(2,1,1);
plot(t,r,'k--',t,back_T.y,t,back.y,'LineWidth',1.4);
grid on; ylabel('Saida relativa');
title('Back-Calculation: integracao Tustin anterior vs Euler regressivo');
legend({'Referencia','Tustin anterior','Euler novo'},'Location','best');
subplot(2,1,2);
plot(t,100*back_T.duty,t,100*back.duty,'LineWidth',1.4);
grid on; xlabel('Tempo (s)'); ylabel('PWM (%)');
legend({'Tustin anterior','Euler novo'},'Location','best');

%% 6) Exportar graficos, tabela e dados
outdir = 'resultados_PI';
if ~exist(outdir,'dir'), mkdir(outdir); end
writetable(resumo,fullfile(outdir,'resumo_metricas.csv'));
saveas(fig1,fullfile(outdir,'01_comparacao_geral.png'));
saveas(fig2,fullfile(outdir,'02_recuperacao.png'));
saveas(fig3,fullfile(outdir,'03_tustin_vs_euler.png'));
save(fullfile(outdir,'simulacoes.mat'),'t','r','G','Gd', ...
     'ideal','sat','back','clamp','back_T','p','resumo');
fprintf('\nArquivos gerados na pasta: %s\n', outdir);

%% FUNCOES LOCAIS
function out = simular_pi(A,B,C,t,r,p,modo)
    N = numel(t);
    x = zeros(size(A,1),1);   % planta parte do ponto de operacao nominal
    I = 0; e_prev = 0;
    out.y = zeros(size(t));
    out.e = zeros(size(t));
    out.duty = zeros(size(t));
    out.requested = zeros(size(t));
    out.I = zeros(size(t));

    for k = 1:N
        % Medida da planta no inicio da amostra k (D=0)
        yk = C*x;
        ek = r(k) - yk;
        Pk = p.Kp*ek;

        % Integracao escolhida para cada cenario.
        if strcmp(modo,'back_tustin')
            I_test = I + p.Ki*p.Ts/2*(ek + e_prev);
        else
            % Euler regressivo - metodo escolhido para o PIC
            I_test = I + p.Ki*p.Ts*ek;
        end

        if strcmp(modo,'clamping')
            % Clamping DIRETO do estado integral: [-0.5,+0.5]
            I_test = min(p.Imax,max(p.Imin,I_test));
        end

        u_req = p.base + Pk + I_test;
        if strcmp(modo,'ideal')
            u_aplicado = u_req;  % sem limite: referencia ideal academica
        else
            u_aplicado = min(p.umax,max(p.umin,u_req));
        end

        if strcmp(modo,'back_euler') || strcmp(modo,'back_tustin')
            % Back-Calculation: corrige o acumulador com erro de saturacao
            I = I_test + p.Kaw*p.Ts*(u_aplicado-u_req);
        else
            I = I_test;
        end

        out.y(k) = yk;
        out.e(k) = ek;
        out.duty(k) = u_aplicado;
        out.requested(k) = u_req;
        out.I(k) = I;

        % A planta recebe o incremento de PWM em torno de 50% nominal.
        % ZOH: entrada constante durante cada periodo de 100 ms.
        x = A*x + B*(u_aplicado - p.base);
        e_prev = ek;
    end
end

function m = metricas_pi(t,r,out)
    % Resposta ao primeiro degrau: 5 <= t < 80 s, referencia -0.30.
    j1 = (t >= 5) & (t < 80);
    tt = t(j1)-5;
    yy = out.y(j1);
    alvo = -0.30;

    t10 = instante_cruzamento(tt,yy,0.10*alvo);
    t90 = instante_cruzamento(tt,yy,0.90*alvo);
    m.subida = t90-t10;

    fora = find(abs(yy-alvo) > 0.02*abs(alvo));
    if isempty(fora)
        m.acomodacao = 0;
    elseif fora(end) == numel(tt)
        m.acomodacao = NaN; % nao acomodou antes de t=80 s
    else
        m.acomodacao = tt(fora(end)+1);
    end
    m.sobressinal = max(0,(alvo-min(yy))/abs(alvo)*100);

    % IAE da recuperacao entre 150 e 240 s.
    j2 = (t >= 150) & (t <= 240);
    m.iae = trapz(t(j2),abs(r(j2)-out.y(j2)));
    m.yfinal = out.y(end);
    m.pwmfinal = 100*out.duty(end);
end

function instante = instante_cruzamento(t,y,limiar)
    % Para o degrau NEGATIVO: procura a primeira passagem para baixo.
    idx = find(y <= limiar,1,'first');
    if isempty(idx)
        instante = NaN;
    elseif idx == 1
        instante = t(1);
    else
        instante = t(idx-1) + (t(idx)-t(idx-1))* ...
            (limiar-y(idx-1))/(y(idx)-y(idx-1));
    end
end
