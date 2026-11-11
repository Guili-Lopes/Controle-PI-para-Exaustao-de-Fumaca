%% RESULTADOS DOS 4 CONTROLADORES A PARTIR DO SIMULINK
% Execute depois de criar 'comparacao_PI_Simulink.slx' com
% Criar_Modelo_PI_Simulink.m.
% Requer MATLAB + Simulink (Control System Toolbox para a criacao do modelo).

clear; clc; close all;
mdl = 'comparacao_PI_Simulink';
if ~bdIsLoaded(mdl)
    load_system(mdl);
end
out = sim(mdl);

ref = out.get('referencia_PI');
modos = {'ideal','saturado','back_calculation','clamping'};
rotulos = {'PI ideal','Saturado sem AW','Back-Calculation','Clamping'};
col = lines(numel(modos));

figure('Name','Simulink - Saida relativa','Color','w');
stairs(ref.Time,ref.Data,'k--','LineWidth',1.4); hold on;

IAE = zeros(numel(modos),1);
yfinal = zeros(numel(modos),1);
pwmfinal = zeros(numel(modos),1);

for j = 1:numel(modos)
    yy = out.get(['y_' modos{j}]);
    dd = out.get(['pwm_' modos{j}]);
    plot(yy.Time,yy.Data,'Color',col(j,:),'LineWidth',1.1);

    % Interpola a referencia em cada instante registrado pela planta.
    r_interp = interp1(ref.Time(:),ref.Data(:),yy.Time(:),'previous','extrap');
    indice = yy.Time >= 150 & yy.Time <= 240;
    IAE(j) = trapz(yy.Time(indice), ...
                   abs(r_interp(indice)-yy.Data(indice)));
    yfinal(j) = yy.Data(end);
    pwmfinal(j) = 100*dd.Data(end);
end
xlabel('Tempo (s)'); ylabel('Saida relativa'); grid on;
title('PI discreto - comparacao no Simulink');
legend([{'Referencia'}, rotulos],'Location','best');

figure('Name','Simulink - Duty PWM','Color','w'); hold on;
for j = 1:numel(modos)
    dd = out.get(['pwm_' modos{j}]);
    plot(dd.Time,100*dd.Data,'Color',col(j,:),'LineWidth',1.1);
end
yline(0,'k:'); yline(100,'k:'); grid on;
xlabel('Tempo (s)'); ylabel('Duty PWM (%)');
title('Atuacao solicitada/aplicada');
legend(rotulos,'Location','best');

figure('Name','Simulink - Integrador','Color','w'); hold on;
for j = 1:numel(modos)
    ii = out.get(['I_' modos{j}]);
    plot(ii.Time,ii.Data,'Color',col(j,:),'LineWidth',1.1);
end
yline(-0.5,'k:'); yline(0.5,'k:'); grid on;
xlabel('Tempo (s)'); ylabel('Integral');
title('Estado do integrador - limites do Clamping');
legend(rotulos,'Location','best');

resumo = table(rotulos', IAE, yfinal, pwmfinal, ...
    'VariableNames',{'Controlador','IAE_150_a_240', ...
                     'Saida_Final','PWM_Final_pct'});
disp(resumo);
fprintf('Reducao de IAE (Back-Calculation vs saturado): %.2f%%\n', ...
    100*(1-IAE(3)/IAE(2)));
fprintf('Reducao de IAE (Clamping vs saturado): %.2f%%\n', ...
    100*(1-IAE(4)/IAE(2)));

outdir = 'resultados_Simulink';
if ~exist(outdir,'dir'), mkdir(outdir); end
figs = findobj('Type','figure');
for k = 1:numel(figs)
    saveas(figs(k),fullfile(outdir,['figura_' num2str(k) '.png']));
end
writetable(resumo,fullfile(outdir,'metricas.csv'));
save(fullfile(outdir,'dados_Simulink.mat'),'out','resumo');
fprintf('Resultados salvos em: %s\n',outdir);
