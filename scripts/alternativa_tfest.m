% alternativa_tfest.m  -  Alternativa con tfest si la lamina no oscila
% Manual de la MPA (2026-2), seccion 8.6. Sistemas de Control en Tiempo Continuo, EIA.
% Se ejecuta despues de analisis_escalones_modelo.m (usa y, u, t, ym y Ts).

w   = t >= tSub-5 & t < tSub+Tesp;
dat = iddata(y(w) - ym(tSub-5,tSub), u(w) - u0, Ts);
G2  = tfest(dat, 2, 0, NaN);            % 2 polos, 0 ceros, retardo libre
[num, den] = tfdata(G2, 'v');           % den = [1  2*zeta*wn  wn^2]
wn = sqrt(den(3));  zeta = den(2)/(2*wn);  K = num(end)/den(3);
