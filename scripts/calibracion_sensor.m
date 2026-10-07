%% calibracion_sensor.m  -  Calibracion del potenciometro de la MPA (cuentas -> grados)
% Manual de la MPA (2026-2), seccion 6. Sistemas de Control en Tiempo Continuo, EIA.
%
% Como se usa:
%   1. Con el motor APAGADO, lleven la lamina a 0, 10, ..., 90 grados (subiendo
%      y luego bajando) y anoten la salida 'adc' del subsistema MPA en la hoja
%      de campo (anexo B del manual). Anoten tambien 15, 45 y 75 grados.
%   2. Copien sus lecturas en la seccion DATOS y ejecuten el archivo (F5).
%   3. El resultado queda en cal_MPAxx.mat con las variables a2, a1, a0.
%      El modelo de Simulink lo carga solo al iniciar (InitFcn) y los bloques
%      Gain/Constant de la calibracion usan esos nombres:
%           y_deg = a2*adc^2 + a1*adc + a0      (a2 = 0 si el ajuste es lineal)

clear; clc; close all

%% DATOS (llenar con SU modulo)
modulo  = 4;                    % numero de la MPA
theta   = 0:10:90;              % marcas del indicador [grados]
n_sub   = [ ];                  % adc subiendo  (10 valores, uno por marca)
n_baj   = [ ];                  % adc bajando   (10 valores, uno por marca)
theta_v = [15 45 75];           % marcas de validacion (NO se usan en el ajuste)
n_v     = [ ];                  % adc en esas marcas (3 valores; puede quedar vacio)

%% Comprobaciones
assert(numel(n_sub) == numel(theta) && numel(n_baj) == numel(theta), ...
    'Llenen n_sub y n_baj con %d valores cada uno.', numel(theta))
n = (n_sub + n_baj) / 2;                       % promedio subiendo/bajando
d = diff(n);
if ~(all(d > 0) || all(d < 0))
    warning(['La lectura no es monotona: revisen el acople eje-potenciometro ', ...
             'antes de seguir (no lo arregla ningun polinomio).'])
end
histeresis = max(abs(n_sub - n_baj));          % [cuentas]
if histeresis > 5
    warning('Histeresis de %.1f cuentas entre subida y bajada: revisen el acople.', histeresis)
end
resolucion = 90 / abs(n(end) - n(1));          % [grados por cuenta]

%% Ajustes lineal y cuadratico
p1 = polyfit(n, theta, 1);   e1 = theta - polyval(p1, n);
p2 = polyfit(n, theta, 2);   e2 = theta - polyval(p2, n);
emax1 = max(abs(e1));        emax2 = max(abs(e2));

if emax1 < 1                                   % la recta basta
    tipo = 'lineal';      a2 = 0;      a1 = p1(1);  a0 = p1(2);  e_ajuste = emax1;
else                                           % hace falta la cuadratica
    tipo = 'cuadratico';  a2 = p2(1);  a1 = p2(2);  a0 = p2(3);  e_ajuste = emax2;
end
p_cal = [a2 a1 a0];                            % polyval(p_cal, adc) = grados

%% Validacion con marcas que no se usaron para ajustar
if ~isempty(n_v)
    e_val = theta_v - polyval(p_cal, n_v);
    e_validacion = max(abs(e_val));
else
    e_val = [];  e_validacion = NaN;
end

%% Graficas
figure('Name', sprintf('Calibracion MPA %d', modulo), 'Color', 'w')
subplot(2,1,1)
nn = linspace(min(n), max(n), 200);
plot(n_sub, theta, '^', n_baj, theta, 'v', 'MarkerSize', 7); hold on
plot(nn, polyval(p1, nn), '--', nn, polyval(p2, nn), '-', 'LineWidth', 1.3)
if ~isempty(n_v)
    plot(n_v, theta_v, 'ks', 'MarkerFaceColor', 'y', 'MarkerSize', 8)
end
grid on; xlabel('adc [cuentas]'); ylabel('\theta [grados]')
title(sprintf('MPA %d: ajuste elegido = %s', modulo, tipo))
leyenda = {'subiendo', 'bajando', ...
           sprintf('lineal (error max %.2f^\\circ)', emax1), ...
           sprintf('cuadratico (error max %.2f^\\circ)', emax2)};
if ~isempty(n_v), leyenda{end+1} = 'validacion'; end
legend(leyenda, 'Location', 'best')

subplot(2,1,2)
stem(theta - 1, e1, 'filled'); hold on
stem(theta + 1, e2, 'filled')
yline(0, 'k'); yline(1, ':'); yline(-1, ':')
grid on; xlabel('\theta [grados]'); ylabel('error [grados]')
legend('lineal', 'cuadratico', 'Location', 'best')
title('Error de ajuste en cada marca (punteadas: \pm1 grado)')

%% Resumen para la ficha
fprintf('\n=== Calibracion MPA %d ===\n', modulo)
fprintf('Ajuste elegido      : %s\n', tipo)
fprintf('  a2 = %+.6e\n  a1 = %+.6e\n  a0 = %+.6e\n', a2, a1, a0)
fprintf('Error de ajuste max : %.2f grados\n', e_ajuste)
fprintf('Error de validacion : %.2f grados\n', e_validacion)
fprintf('Histeresis max      : %.1f cuentas\n', histeresis)
fprintf('Resolucion          : %.2f grados por cuenta\n', resolucion)
if n(end) > n(1)
    fprintf('Sentido             : adc AUMENTA al subir la lamina\n')
else
    fprintf('Sentido             : adc DISMINUYE al subir la lamina\n')
end

%% Guardar (lo carga el modelo de Simulink)
archivo = sprintf('cal_MPA%02d.mat', modulo);
fecha   = datestr(now, 'yyyy-mm-dd HH:MM');
save(archivo, 'a2', 'a1', 'a0', 'p_cal', 'tipo', 'e_ajuste', 'e_validacion', ...
     'histeresis', 'resolucion', 'theta', 'n_sub', 'n_baj', 'theta_v', 'n_v', ...
     'modulo', 'fecha')
fprintf('\nGuardado en %s\n', archivo)
fprintf('En Simulink: pestana Modeling > Model Settings (flecha) > Model Properties > Callbacks > InitFcn:\n')
fprintf('    load(''%s'')\n', archivo)
