%% programa_escalera.m  -  Programa del ensayo en escalera (mapa estatico de la MPA)
% Manual de la MPA (2026-2), seccion 7. Sistemas de Control en Tiempo Continuo, EIA.
%
% Se ejecuta ANTES del Run (F5). Arma el programa de mando u_prog, que lee el
% bloque From Workspace del modelo, y fija el Stop time.
% Despues del Run se ejecuta analisis_escalera.m.

% Borra los datos de una corrida anterior para no analizarlos por error
clear y_deg u_ap t_ard latencia t tout

modulo = 4;                  % numero de SU MPA
modelo = 'MPA_04_ensayos';   % nombre del .slx (sin extension)

uini = 6;     % [%]  primer escalon: un poco por debajo de u_mov (primera prueba)
ufin = 90;    % [%]  ultimo escalon: u con la lamina cerca de 85 grados
du   = 6;     % [%]  tamano del escalon (4 si quieren mas detalle)
Th   = 10;    % [s]  duracion de cada escalon (15 s si la lamina sigue oscilando)
Tprom = 3;    % [s]  ventana final de cada escalon que se promedia (ya asentada)
Tarm = 20;    % [s]  u = 0 al inicio: ENCENDER LA FUENTE en estos segundos

niv  = [uini:du:ufin, ufin-du:-du:uini];          % sube y baja
tniv = Tarm + (0:numel(niv)-1)*Th;                % inicio de cada escalon
ev   = [0 0; tniv' niv'; tniv(end)+Th 0];         % [tiempo  u]; al final u = 0
u_prog = timeseries(ev(:,2), ev(:,1));
Tfin = tniv(end) + Th + 5;                        % Stop time
if Th - Tprom < 6
    warning(['Th = %g s es corto: la lamina necesita unos 6 s para asentarse antes de ', ...
             'la ventana de %g s que se promedia.'], Th, Tprom)
end

try                                                % si el modelo esta abierto,
    set_param(modelo, 'StopTime', num2str(Tfin));  % se le pone el Stop time
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
fprintf('%d escalones, duracion %.1f min. Enciendan la fuente en los primeros %d s.\n', ...
        numel(niv), Tfin/60, Tarm)
fprintf('Ahora: Run en Simulink. Al terminar, ejecuten analisis_escalera.m\n')
