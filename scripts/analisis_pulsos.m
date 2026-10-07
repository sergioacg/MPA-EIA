% analisis_pulsos.m  -  Polos (wn, zeta) a partir de la respuesta libre tras los pulsos
% Manual de la MPA (2026-2), seccion 8.3. Sistemas de Control en Tiempo Continuo, EIA.
% Se ejecuta DESPUES del Run, con las variables de programa_identificacion.m.
% Lee del workspace las variables sueltas de los To Workspace (formato Array,
% 'Single simulation output' desmarcado): y_deg, u_ap y t_ard.

assert(exist('y_deg', 'var') == 1 && exist('u_ap', 'var') == 1 && exist('t_ard', 'var') == 1, ...
    'No estan y_deg, u_ap y t_ard en el workspace: corran el ensayo primero.')
assert(exist('tPul', 'var') == 1, 'Falta el programa: ejecuten programa_identificacion.m.')
Ts = 0.02;                                           % paso fijo del modelo [s]
y  = double(y_deg(:));   N = numel(y);
if exist('t', 'var') == 1 && numel(t) == N          % To Workspace 't' del Clock
    t = double(t(:));
elseif exist('tout', 'var') == 1 && numel(tout) == N
    t = tout(:);                                     % tiempo de simulacion
else
    t = (0:N-1)' * Ts;                               % mismo tiempo, armado con Ts
end
if t(end) < Tfin - 1
    error('Los datos duran %.1f s y el programa %.1f s: la corrida se detuvo antes.', t(end), Tfin)
elseif t(end) > Tfin + 1
    warning('Los datos duran %.1f s y el programa %.1f s: revisen que sean de esta corrida.', t(end), Tfin)
end
% Los To Workspace se sobrescriben en el siguiente Run: se guarda la corrida
save(sprintf('ident_MPA%02d_%s.mat', modulo, datestr(now, 'yyyymmdd_HHMM')), ...
     'y_deg', 'u_ap', 't_ard', 't', 'ev', 'tPul', 'tSub', 'tBaj', 'u0', 'Ap', ...
     'tp', 'dU', 'Tesp', 'rampa', 'Tfin', 'modulo')

% --- Respuesta libre: despues de que termina el pulso, el mando queda fijo en
% u0 y la lamina se mueve sola. Esa respuesta depende solo de la matriz A:
%     dtheta(t) = Am * exp(-sigma*t) * cos(wd*t + fase) + c
% Se ajustan los 5 parametros a TODOS los puntos de la respuesta libre (no
% solo a los picos): con zeta ~ 0.3 la lamina da apenas uno o dos picos
% visibles antes de perderse en el ruido.
Tlib = 6;                                            % [s] de respuesta libre que se ajustan
opc  = optimset('Display', 'off', 'MaxFunEvals', 5000, 'MaxIter', 5000);
res  = zeros(numel(tPul), 4);                        % [sigma  wd  fit  pico]
figure('Name', 'Respuesta libre tras cada pulso', 'Color', 'w')
for k = 1:numel(tPul)
    y0 = mean(y(t >= tPul(k)-5 & t < tPul(k)));      % equilibrio antes del pulso
    tl = tPul(k) + tp + Ap/rampa + 0.05;             % termino la rampa de bajada
    w  = t >= tl & t < tl + Tlib;                    % respuesta libre
    tt = t(w) - tl;   z = y(w) - y0;
    libre = @(p) p(1)*exp(-p(2)*tt).*cos(p(3)*tt + p(4)) + p(5);
    J     = @(p) sum((z - libre(p)).^2);
    Jmin = inf;
    for wd0 = [3 5 7 9 12]                           % varios arranques en wd
        [p, Jp] = fminsearch(J, [z(1) 2 wd0 0 0], opc);
        if Jp < Jmin, Jmin = Jp; pb = p; end
    end
    sig = pb(2);   wdk = abs(pb(3));
    fitk = 100*(1 - sqrt(Jmin)/norm(z - mean(z)));
    res(k,:) = [sig, wdk, fitk, max(abs(z))];
    % --- grafica: medida, modelo ajustado y envolvente
    subplot(numel(tPul), 1, k)
    wv = t >= tPul(k)-2 & t < tl + Tlib;
    plot(t(wv)-tPul(k), y(wv)-y0, 'Color', [0.6 0.6 0.6]); hold on
    plot(tt+tl-tPul(k), libre(pb), 'b', 'LineWidth', 1.4)
    env = abs(pb(1))*exp(-sig*tt);
    plot(tt+tl-tPul(k), pb(5)+env, 'r--', tt+tl-tPul(k), pb(5)-env, 'r--')
    grid on; xlim([-2 tl+Tlib-tPul(k)]); ylabel('\Delta\theta [grados]')
    title(sprintf('Pulso %d: \\omega_n = %.2f rad/s, \\zeta = %.3f, fit = %.0f %%', ...
                  k, hypot(sig, wdk), sig/hypot(sig, wdk), fitk))
    if sig <= 0
        warning('Pulso %d: la envolvente no decae (sigma <= 0). Revisen la grafica.', k)
    elseif fitk < 70
        warning('Pulso %d: el modelo explica poco la respuesta (fit %.0f %%). Revisen la grafica.', k, fitk)
    end
    if res(k,4) < 3
        warning('Pulso %d: respuesta de solo %.1f grados; suban Ap o tp.', k, res(k,4))
    end
end
xlabel('t desde el inicio del pulso [s]   (gris: medida; azul: modelo ajustado; rojo: envolvente)')

wn_k = hypot(res(:,1), res(:,2));   zeta_k = res(:,1)./wn_k;
disp(array2table([res(:,1:2) wn_k zeta_k res(:,3)], ...
     'VariableNames', {'sigma','wd','wn','zeta','fit'}))
ok = res(:,1) > 0;
sigma = mean(res(ok,1));  wd = mean(res(ok,2));
wn = hypot(sigma, wd);    zeta = sigma/wn;
fprintf('wn = %.2f rad/s   zeta = %.3f\n', wn, zeta)
if (max(wn_k(ok)) - min(wn_k(ok)))/2 > 0.10*wn
    warning('Los pulsos no coinciden en wn (mas de +-10 %%): miren cual se sale y por que.')
end
