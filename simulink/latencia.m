function lat = latencia(seq, ack)
%#codegen
% latencia  Pasos entre el ultimo comando enviado y el ultimo que acepto el Arduino.
% Entradas: seq (salida de packMPA), ack (salida de parseMPA). Normal: 0 o 1.
persistent s_ant
if isempty(s_ant), s_ant = 0; end
lat = mod(s_ant - ack, 65536);
s_ant = seq;
end
