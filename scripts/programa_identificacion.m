% programa_identificacion.m  -  Programa del ensayo de identificacion (pulsos y escalones)
% Manual de la MPA (2026-2), seccion 8.2. Sistemas de Control en Tiempo Continuo, EIA.
% Se ejecuta ANTES del Run: arma u_prog (From Workspace) y fija el Stop time.
% Despues del Run sigue analisis_pulsos.m y luego analisis_escalones_modelo.m.

% Borra los datos de una corrida anterior para no analizarlos por error
clear y_deg u_ap t_ard latencia t tout

modulo = 4;   %      numero de SU MPA
modelo = 'MPA_04_ensayos';   % nombre del .slx (sin extension)
u0   = 40;    % [%]  mando del punto de operacion (del mapa estatico)
Ap   = 10;    % [%]  amplitud del pulso
tp   = 0.4;   % [s]  duracion del pulso
dU   = 4;     % [%]  escalon pequeno
Tarm = 20;    % [s]  u = 0 para armar el ESC (encender la fuente aqui)
Tesp = 30;    % [s]  espera entre eventos
rampa = 50;   % [%/s] RAMPA_PCT_S del firmware
ev = [0 0; Tarm u0];
t  = Tarm + 2*Tesp;                          % 60 s para asentarse en theta0
tPul = zeros(1,3);
for k = 1:3
    tPul(k) = t;
    ev = [ev; t u0+Ap; t+tp u0];   t = t + Tesp;
end
tSub = t;  ev = [ev; t u0+dU];   t = t + Tesp;   % escalon arriba (estimacion)
           ev = [ev; t u0];      t = t + Tesp;   % regreso
tBaj = t;  ev = [ev; t u0-dU];   t = t + Tesp;   % escalon abajo (validacion)
           ev = [ev; t u0];      t = t + Tesp;
ev = [ev; t 0];                                  % fin: motor a cero
u_prog = timeseries(ev(:,2), ev(:,1));
Tfin = t + 5;                                    % Stop time

try                                              % si el modelo esta abierto,
    set_param(modelo, 'StopTime', num2str(Tfin));% se le pone el Stop time
    fprintf('Stop time de %s = %g s\n', modelo, Tfin)
catch
    fprintf('Pongan Stop time = %g s en el modelo\n', Tfin)
end
% Configura el From Workspace que lee u_prog: sin interpolar (escalones
% limpios), Sample time 0.02 y mantener el ultimo valor al terminar
try
    b = find_system(modelo, 'LookUnderMasks', 'all', 'BlockType', 'FromWorkspace');
    for k = 1:numel(b)
        if strcmp(get_param(b{k}, 'VariableName'), 'u_prog')
            set_param(b{k}, 'Interpolate', 'off', 'SampleTime', '0.02', ...
                      'OutputAfterFinalValue', 'Holding final value')
        end
    end
catch
    fprintf('Revisen a mano el From Workspace: Interpolate data DESMARCADO, Sample time 0.02.\n')
end
fprintf('Duracion %.1f min. Enciendan la fuente en los primeros %d s.\n', Tfin/60, Tarm)
