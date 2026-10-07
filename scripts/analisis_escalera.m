%% analisis_escalera.m  -  Analisis del ensayo en escalera (mapa estatico de la MPA)
% Manual de la MPA (2026-2), seccion 7. Sistemas de Control en Tiempo Continuo, EIA.
%
% Se ejecuta DESPUES del Run (F5), con el programa de programa_escalera.m en el
% workspace. Calcula el mapa theta_ss(u), los mandos para 30/45/60 grados, la
% ganancia local y la histeresis, y guarda todo en mapa_MPAxx_fecha.mat.
%
% Lo que espera del modelo:
%   - Data Import/Export: 'Single simulation output' DESMARCADO, asi cada
%     To Workspace deja su variable suelta en el workspace (sin 'out').
%   - To Workspace en formato Array, Sample time -1:
%       y_deg (angulo calibrado), u_ap (mando aplicado), t_ard (reloj del
%       Arduino) y, si existe, latencia.
%   - El tiempo se toma del To Workspace 't' (bloque Clock) o de 'tout'; si
%     no esta ninguno, se arma con el paso fijo Ts = 0.02 s.

assert(exist('niv', 'var') == 1 && exist('Tfin', 'var') == 1, ...
    'Falta el programa del ensayo: ejecuten programa_escalera.m antes del Run.')
if exist('y_deg', 'var') ~= 1 || exist('u_ap', 'var') ~= 1
    if exist('out', 'var') == 1
        error(['Los datos quedaron dentro de ''out'': desmarquen ''Single simulation ', ...
               'output'' en Model Settings > Data Import/Export y corran de nuevo.'])
    end
    error('No estan y_deg y u_ap en el workspace: corran el ensayo (Run) primero.')
end

% --- 0. Datos (To Workspace en formato Array: un valor por paso de 0.02 s)
Ts = 0.02;                                         % paso fijo del modelo [s]
y  = double(y_deg(:));    N = numel(y);
ua = double(u_ap(:));
if exist('t', 'var') == 1 && numel(t) == N        % To Workspace 't' del Clock
    t = double(t(:));
elseif exist('tout', 'var') == 1 && numel(tout) == N
    t = tout(:);                                   % tiempo de simulacion
else
    t = (0:N-1)' * Ts;                             % mismo tiempo, armado con Ts
end
if t(end) < Tfin - 1
    error(['Los datos duran %.1f s y el ensayo programado %.1f s: la corrida se ', ...
           'detuvo antes o los datos son de otra corrida.'], t(end), Tfin)
elseif t(end) > Tfin + 1
    warning('Los datos duran %.1f s y el ensayo programado %.1f s: revisen que sean de esta corrida.', t(end), Tfin)
end

% --- 1. Calidad del enlace durante el ensayo
if exist('t_ard', 'var') == 1
    ta  = double(t_ard(:));
    dta = diff(ta);
    fprintf('\nReloj del Arduino: paso min %.3f s, max %.3f s (debe ser 0.020)\n', min(dta), max(dta))
    if numel(ta) ~= N
        warning(['t_ard tiene %d muestras y y_deg %d: revisen que todos los To Workspace ', ...
                 'tengan Sample time -1 y Decimation 1.'], numel(ta), N)
    else
        r = (ta(end) - ta(1)) / (t(end) - t(1));
        fprintf('Tiempo del Arduino / tiempo de Simulink: %.2f (debe ser 1.00)\n', r)
        if r > 1.05
            warning(['Simulink avanzo %.1f veces mas lento que el Arduino: cada paso ', ...
                     'consumio mas de un paquete. Revisen Serial Receive (Data size [22 1], ', ...
                     'Sample time 0.02), el paso fijo (0.02) y cierren Scopes y Displays.'], r)
        end
    end
end
if exist('latencia', 'var') == 1
    lat = double(latencia(:));
    fprintf('Latencia maxima: %d pasos (debe ser 0 o 1)\n', max(lat(10:end)))
end

% --- 2. Estado estacionario de cada escalon (ultimos Tprom segundos)
if exist('Tprom', 'var') ~= 1, Tprom = 5; end
yss = zeros(size(niv));   ysd = zeros(size(niv));   usd = zeros(size(niv));
for k = 1:numel(niv)
    w = t >= tniv(k)+Th-Tprom & t < tniv(k)+Th;
    yss(k) = mean(y(w));
    ysd(k) = std(y(w));                            % si es grande, no se asento
    usd(k) = std(ua(w));                           % el mando debe ser constante
end
if any(usd > 0.2)
    warning(['El mando no fue constante dentro de los escalones: en el bloque From ', ...
             'Workspace desmarquen ''Interpolate data''.'])
end
inquietos = find(ysd > 1);
if ~isempty(inquietos)
    warning('En %d escalones la lamina seguia moviendose (desv. > 1 grado). Suban Th.', numel(inquietos))
end

ns   = ceil(numel(niv)/2);                         % escalon mas alto
uS   = niv(1:ns);        yS = yss(1:ns);           % rama subiendo
uB   = niv(ns:end);      yB = yss(ns:end);         % rama bajando

% --- 3. Histeresis: diferencia subiendo/bajando en el mismo u
[uc, iS, iB] = intersect(uS, uB);
histeresis = abs(yS(iS) - yB(iB));
[hmax, ih] = max(histeresis);
fprintf('Histeresis maxima: %.1f grados (en u = %g %%)\n', hmax, uc(ih))

% --- 4. u_mov: primer escalon (subiendo) en que la lamina se separa del tope
y_reposo = yS(1);
i_mov = find(yS > y_reposo + 2, 1);
if isempty(i_mov)
    warning('La lamina no se separo del tope en ningun escalon: revisen la helice (seccion 4).')
    i_mov = 1;
else
    fprintf('u_mov aprox.: entre %g y %g %%\n', uS(max(i_mov-1,1)), uS(i_mov))
end

% --- 5. Mando para 30, 45 y 60 grados (rama subiendo, tramo creciente)
objetivos = [30 45 60];
[ySu, ia] = unique(yS(i_mov:end));                 % interp1 pide valores unicos
uSu = uS(i_mov - 1 + ia);
u_obj = interp1(ySu, uSu, objetivos, 'linear', NaN);
for j = 1:numel(objetivos)
    fprintf('u para %2d grados: %5.1f %%\n', objetivos(j), u_obj(j))
end

% --- 6. Ganancia local K = d(theta_ss)/du en la rama subiendo
K_loc = gradient(yS, uS);                          % [grados/%]

% --- 7. Graficas
figure('Name', sprintf('Mapa estatico MPA %d', modulo), 'Color', 'w')
subplot(2,1,1)
plot(uS, yS, 'o-', uB, yB, 's--', 'LineWidth', 1.2); hold on
plot(u_obj, objetivos, 'kx', 'MarkerSize', 10, 'LineWidth', 1.5)
grid on; xlabel('u [%]'); ylabel('\theta_{ss} [grados]')
legend('subiendo', 'bajando', 'u para 30/45/60', 'Location', 'best')
title(sprintf('MPA %d: mapa estatico', modulo))
subplot(2,1,2)
plot(uS, K_loc, 'o-', 'LineWidth', 1.2); grid on
xlabel('u [%]'); ylabel('K local [grados/%]')
title('Ganancia local (pendiente del mapa, rama subiendo)')

figure('Name', 'Ensayo en el tiempo', 'Color', 'w')
subplot(2,1,1); plot(t, y); grid on; ylabel('\theta [grados]')
subplot(2,1,2); plot(t, ua); grid on; ylabel('u_{ap} [%]'); xlabel('t [s]')

% --- 8. Guardar
archivo = sprintf('mapa_MPA%02d_%s.mat', modulo, datestr(now, 'yyyymmdd_HHMM'));
save(archivo, 'niv', 'tniv', 'Th', 'Tprom', 'yss', 'ysd', 'uS', 'yS', 'uB', 'yB', ...
     'histeresis', 'u_obj', 'objetivos', 'K_loc', 'modulo', 't', 'y', 'ua', 'Tfin')
if exist('t_ard', 'var') == 1, save(archivo, 't_ard', '-append'); end   % reloj real
fprintf('\nGuardado en %s\n', archivo)
