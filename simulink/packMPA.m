function [trama, seq] = packMPA(u, habilitar)
%#codegen
% packMPA  Arma el comando para el MPA (bloque MATLAB Function).
%
% Entradas: u          mando [%] (despues de la saturacion del modelo)
%           habilitar  1 = marcha permitida, 0 = PARO (motor detenido)
% Salidas:  trama      11x1 uint8 hacia el bloque Serial Send
%                      (Header vacio, Terminator <none>)
%           seq        numero de este comando (para medir la latencia con ack)
%
% Trama: 0xCC 0xDD | seq uint16 | u single | flags uint8 | crc16 (little-endian)
% El Arduino recorta u a [0,100] y tiene su propio tope de pulso; aqui
% tambien se recorta para que NaN o valores absurdos nunca salgan del PC.

persistent s
if isempty(s)
    s = 0;
end
s = mod(s + 1, 65536);                 % uint16 en MATLAB satura; aqui se envuelve

if ~isfinite(u)
    u = 0;
end
u = min(max(u, 0), 100);
fl = uint8(0);
if habilitar > 0.5
    fl = uint8(1);
end

carga = [typecast(uint16(s), 'uint8'), typecast(single(u), 'uint8'), fl];   % 7 bytes
trama = zeros(11, 1, 'uint8');
trama(1) = uint8(204);                 % 0xCC
trama(2) = uint8(221);                 % 0xDD
for k = 1:7
    trama(2+k) = carga(k);
end
c = crc16(carga);
trama(10) = uint8(mod(c, 256));
trama(11) = uint8(floor(c / 256));
seq = s;
end

% ---------------------------------------------------------------------
function c = crc16(d)
% CRC-16/CCITT-FALSE: polinomio 0x1021, inicial 0xFFFF (el mismo del Arduino)
r = uint16(65535);
for k = 1:numel(d)
    r = bitxor(r, bitshift(uint16(d(k)), 8));
    for b = 1:8
        if bitand(r, uint16(32768)) ~= 0
            r = bitxor(bitshift(r, 1), uint16(4129));
        else
            r = bitshift(r, 1);
        end
    end
end
c = double(r);
end
