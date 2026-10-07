function [adc, u_ap, t_ard, seq, ack, estado, valido, edad, n_malos] = parseMPA(raw)
%#codegen
% parseMPA  Decodifica la telemetria del MPA (bloque MATLAB Function).
%
% Entrada:  raw  22x1 uint8, salida del bloque Serial Receive
%           (Header vacio, Terminator <none>, Data size [22 1], uint8,
%            modo bloqueante, Block sample time = Ts = 0.02).
% Salidas:  adc      promedio de cuentas de A0 (va a la calibracion)
%           u_ap     mando que el Arduino aplico de verdad [%]
%           t_ard    reloj del Arduino [s] (tiempo real del ensayo)
%           seq      numero del paquete leido
%           ack      ultimo comando que el Arduino acepto (para la latencia)
%           estado   1 = MARCHA, 0 = SEGURO (motor detenido)
%           valido   1 si hay paquete reciente; 0 si las salidas son el ultimo
%                    valor retenido (el lazo no debe confiar en esa medida)
%           edad     pasos seguidos sin paquete nuevo
%           n_malos  lecturas en que habia cabecera pero ningun paquete con
%                    CRC correcto (desde el inicio). Debe quedarse en 0 o casi.
%
% No supone que los 22 bytes leidos esten alineados con un paquete: guarda
% los ultimos 64 bytes y busca, de atras hacia adelante, el paquete mas
% reciente con cabecera 0xAA 0xBB y CRC-16 correcto. Asi se recupera solo
% de un arranque a mitad de paquete o de un byte perdido.

L  = 22;          % largo del paquete
NB = 64;          % memoria de bytes (cabe un paquete completo en cualquier desfase)
EDAD_MAX = 5;     % pasos sin paquete nuevo antes de declarar invalido (0.1 s)

persistent buf adc_m u_m t_m seq_m ack_m est_m nmal ed hay
if isempty(buf)
    buf  = zeros(NB, 1, 'uint8');
    adc_m = 0; u_m = 0; t_m = 0; seq_m = -1; ack_m = 0; est_m = 0;
    nmal = 0; ed = 0; hay = false;
end

% --- desplazar la memoria y agregar los bytes nuevos
r = uint8(raw(:));
for k = 1:NB-L
    buf(k) = buf(k+L);
end
for k = 1:L
    buf(NB-L+k) = r(k);
end

% --- buscar el paquete valido mas reciente
f = zeros(L, 1, 'uint8');
encontrado = false;
candidatos = 0;
for i = NB-L+1:-1:1
    if buf(i) == 170 && buf(i+1) == 187
        candidatos = candidatos + 1;
        for k = 1:L
            f(k) = buf(i+k-1);
        end
        if crc16(f(3:L-2)) == double(f(L-1)) + 256*double(f(L))
            encontrado = true;
            break
        end
    end
end
if candidatos > 0 && ~encontrado
    nmal = nmal + 1;
end

nuevo = false;
if encontrado
    sq = double(f(3)) + 256*double(f(4));                  % uint16, little-endian
    if sq ~= seq_m                      % paquete distinto al ya entregado
        nuevo = true;
        seq_m = sq;
        t_m   = (double(f(5)) + 256*double(f(6)) + 65536*double(f(7)) ...
                 + 16777216*double(f(8))) / 1000;           % uint32 [ms] -> s
        adc_m = bytesAFloat(f(9), f(10), f(11), f(12));
        u_m   = bytesAFloat(f(13), f(14), f(15), f(16));
        ack_m = double(f(17)) + 256*double(f(18));
        est_m = double(bitand(f(19), uint8(1)));
        hay   = true;
    end
end

if nuevo
    ed = 0;
else
    ed = ed + 1;
end

adc = adc_m;  u_ap = u_m;  t_ard = t_m;
seq = max(seq_m, 0);  ack = ack_m;  estado = est_m;
valido = double(hay && ed <= EDAD_MAX);
edad = ed;  n_malos = nmal;
end

% ---------------------------------------------------------------------
function x = bytesAFloat(b1, b2, b3, b4)
% 4 bytes little-endian -> single -> double (escalar fijo para el generador de codigo)
v = zeros(1, 4, 'uint8');
v(1) = b1;  v(2) = b2;  v(3) = b3;  v(4) = b4;
y = typecast(v, 'single');
x = double(y(1));
end

% ---------------------------------------------------------------------
function c = crc16(d)
% CRC-16/CCITT-FALSE: polinomio 0x1021, inicial 0xFFFF (el mismo del Arduino)
r = uint16(65535);
for k = 1:numel(d)
    r = bitxor(r, bitshift(uint16(d(k)), 8));
    for b = 1:8
        if bitand(r, uint16(32768)) ~= 0
            r = bitxor(bitshift(r, 1), uint16(4129));   % 0x1021; en uint16 los bits que salen se pierden
        else
            r = bitshift(r, 1);
        end
    end
end
c = double(r);
end
