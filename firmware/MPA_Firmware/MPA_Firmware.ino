// =====================================================================
//  MPA - Firmware de enlace serial con Simulink (sin Connected IO)
//  Sistemas de Control en Tiempo Continuo - Universidad EIA
//  Prof. Sergio Andres Castano Giraldo
//
//  El Arduino es el RELOJ del ensayo: cada PERIODO_MS lee el potenciometro
//  y envia un paquete de 22 bytes. Simulink lee en modo bloqueante, asi que
//  avanza exactamente un paso por paquete. Simulink responde con un comando
//  de 11 bytes. El firmware nunca confia en el PC:
//    - comandos con CRC-16; los que fallan se descartan;
//    - u fuera de [0,100] se recorta, NaN/Inf se descartan;
//    - el pulso del ESC tiene un tope duro (PULSO_TOPE);
//    - sin comandos validos por TIMEOUT_MS -> motor detenido;
//    - despues de arrancar, de un timeout o de un PARO, el motor solo se
//      rearma con un comando u < U_REARME (como el armado del ESC).
//
//  Placas probadas en el diseno: Arduino UNO y Leonardo. No hace falta el
//  Support Package de Arduino de Simulink. OJO: si alguien abre un modelo
//  en Connected IO o Run on board, Simulink reemplaza este firmware.
// =====================================================================
#include <Arduino.h>
#include <Servo.h>
#include <string.h>
#include <math.h>

// ---------------- Configuracion del modulo (ver ficha) ----------------
const uint8_t  PIN_ESC     = 6;       // pin de senal del ESC de SU modulo
const uint8_t  PIN_POT     = A0;      // potenciometro
const uint32_t BAUD        = 115200;  // igual al Serial Configuration de Simulink
const uint32_t PERIODO_US  = 20000UL; // 50 Hz = Ts de Simulink (0.02 s)
// Escala del mando (senal de servo de 20 ms; el ESC acepta 1000-2000 us = 5-10 %).
// En esta MPA el motor llega a su maxima velocidad cerca de 1450-1500 us, asi que
// 0-100 % se reparte SOLO en la franja util, para usar toda la escala:
//   u =   0 %  -> 1000 us = 5.0 % de duty (motor detenido / armado del ESC)
//   u = 100 %  -> PULSO_100 (maxima velocidad util)
// Si en otra unidad la maxima velocidad llega antes o despues, se ajusta PULSO_100.
const uint16_t PULSO_0     = 1000;    // [us] u = 0 %
const uint16_t PULSO_100   = 1450;    // [us] u = 100 %
// Tope de seguridad: ningun pulso lo supera, aunque alguien cambie la escala.
// Por defecto coincide con PULSO_100 (u = 100 %). El manual de 2016 reporta un
// rango util de 1180-1420 us; con mas pulso la fuente ATX puede protegerse.
const uint16_t PULSO_TOPE  = 1450;    // [us] ningun comando lo supera
// Rampa del actuador: el mando aplicado cambia como maximo RAMPA_PCT_S por
// segundo, subiendo y bajando (tambien en PARO y timeout). Evita los picos de
// corriente al acelerar y la energia que el motor devuelve a la fuente al
// frenar, que es lo que dispara la proteccion de una fuente ATX.
// 0 = sin rampa. Es una no linealidad del actuador: va en el modelo simulado.
const float    RAMPA_PCT_S = 50.0f;   // [%/s] -> 1 % por paso de 20 ms
static_assert(PULSO_0 >= 500 && PULSO_100 <= 2500 && PULSO_0 < PULSO_100, "Revise PULSO_0 y PULSO_100");
static_assert(PULSO_TOPE >= PULSO_0 && PULSO_TOPE <= PULSO_100, "PULSO_TOPE debe estar entre PULSO_0 y PULSO_100");
const uint16_t TIMEOUT_MS  = 200;     // sin comandos validos -> motor detenido
const float    U_REARME    = 1.0f;    // [%] para pasar de SEGURO a MARCHA
const uint8_t  N_ADC       = 16;      // lecturas promediadas por muestra
const uint8_t  VERSION_FW  = 2;       // byte 19 de la telemetria

// ---------------- Protocolo ----------------
// Arduino -> PC (22 bytes, little-endian):
//  [0]  0xAA  [1] 0xBB
//  [2..3]   uint16 seq       contador de paquetes (desborda en 65535 -> 0)
//  [4..7]   uint32 t_ms      reloj del Arduino al tomar la muestra
//  [8..11]  float  adc       promedio de N_ADC lecturas de A0 [cuentas]
//  [12..15] float  u_ap      mando realmente aplicado [%] (despues de rampa y tope)
//  [16..17] uint16 ack       seq del ultimo comando valido recibido
//  [18]     uint8  flags     b0 MARCHA, b1 hubo timeout, b2 hubo atraso del
//                            reloj, b3 paquete no enviado
//  [19]     uint8  version   VERSION_FW
//  [20..21] uint16 crc16     CRC-16/CCITT-FALSE de los bytes 2..19
// PC -> Arduino (11 bytes):
//  [0] 0xCC  [1] 0xDD
//  [2..3] uint16 seq   [4..7] float u [%]   [8] uint8 flags (b0 = habilitar)
//  [9..10] uint16 crc16 de los bytes 2..8
// CRC-16/CCITT-FALSE: polinomio 0x1021, valor inicial 0xFFFF ("123456789" -> 0x29B1).
// Con CRC-8 un paquete corrupto pasaba ~1 de cada 1000 fallas; con CRC-16, ~1 de 65000.
struct __attribute__((packed)) Telemetria {
  uint8_t  h1, h2;
  uint16_t seq;
  uint32_t t_ms;
  float    adc;
  float    u_ap;
  uint16_t ack;
  uint8_t  flags;
  uint8_t  version;
  uint16_t crc;
};
static_assert(sizeof(Telemetria) == 22, "La telemetria debe medir 22 bytes");
static_assert(sizeof(float) == 4, "float debe ser de 4 bytes");

const uint8_t LARGO_CMD = 11;

// ---------------- Estado ----------------
Servo esc;
enum Estado : uint8_t { SEGURO = 0, MARCHA = 1 };
Estado   estado      = SEGURO;
float    u_cmd       = 0.0f;   // ultimo u valido recibido
float    u_ap        = 0.0f;   // u aplicado (sigue a u_obj con la rampa)
float    u_obj       = 0.0f;   // u objetivo (u_cmd en MARCHA, 0 en SEGURO)
uint16_t pulso_ap    = PULSO_0;
uint16_t seq_tx      = 0;
uint16_t ack         = 0;
uint32_t t_ultimoCmd = 0;
bool     huboTimeout = false;
bool     huboAtraso  = false;
bool     huboDescarte = false;
uint32_t t_prox      = 0;

// Recepcion
enum RxEstado : uint8_t { ESPERA_H1, ESPERA_H2, CARGA };
RxEstado rx      = ESPERA_H1;
uint8_t  rxBuf[LARGO_CMD - 2];   // seq(2) + u(4) + flags(1) + crc16(2)
uint8_t  rxN     = 0;

// ---------------- Utilidades ----------------
uint16_t crc16(const uint8_t *d, uint8_t n) {
  uint16_t c = 0xFFFF;
  for (uint8_t i = 0; i < n; i++) {
    c ^= (uint16_t)d[i] << 8;
    for (uint8_t b = 0; b < 8; b++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}

uint16_t pulsoDe(float u) {
  if (u < 0.0f)   u = 0.0f;
  if (u > 100.0f) u = 100.0f;
  float p = PULSO_0 + u * 0.01f * (float)(PULSO_100 - PULSO_0);
  uint16_t pi = (uint16_t)(p + 0.5f);
  if (pi < PULSO_0)    pi = PULSO_0;
  if (pi > PULSO_TOPE) pi = PULSO_TOPE;  // tope duro, pase lo que pase
  return pi;
}

// Fija el objetivo; el pulso cambia en rampa() cada periodo
void aplicar() {
  // u maximo que permite el tope: asi u_ap reporta lo que de verdad recibe el ESC
  const float U_TOPE = 100.0f * (float)(PULSO_TOPE - PULSO_0) / (float)(PULSO_100 - PULSO_0);
  u_obj = (estado == MARCHA) ? u_cmd : 0.0f;
  if (u_obj > U_TOPE) u_obj = U_TOPE;
}

void rampa() {
  if (RAMPA_PCT_S <= 0.0f) {
    u_ap = u_obj;
  } else {
    const float paso = RAMPA_PCT_S * (float)PERIODO_US * 1e-6f;
    float d = u_obj - u_ap;
    if (d >  paso) d =  paso;
    if (d < -paso) d = -paso;
    u_ap += d;
  }
  pulso_ap = pulsoDe(u_ap);
  esc.writeMicroseconds(pulso_ap);
}

void irASeguro() {
  estado = SEGURO;
  u_cmd  = 0.0f;
  aplicar();
}

// Procesa un comando completo (rxBuf = seq, u, flags, crc16)
void procesarComando() {
  uint16_t c;
  memcpy(&c, &rxBuf[7], 2);
  if (crc16(rxBuf, 7) != c) return;                                // CRC malo: se ignora
  uint16_t s;  float u;  uint8_t f = rxBuf[6];
  memcpy(&s, &rxBuf[0], 2);
  memcpy(&u, &rxBuf[2], 4);
  if (!isfinite(u)) return;                                        // NaN / Inf: se ignora
  if (u < 0.0f)   u = 0.0f;
  if (u > 100.0f) u = 100.0f;

  ack = s;
  t_ultimoCmd = millis();
  bool habilitar = (f & 0x01) != 0;

  if (!habilitar) { irASeguro(); return; }                         // PARO desde Simulink
  if (estado == SEGURO) {
    if (u < U_REARME) { estado = MARCHA; huboTimeout = false; }    // rearme solo con u ~ 0
    else return;                                                   // sigue SEGURO
  }
  u_cmd = u;
  aplicar();                                                       // la rampa lo aplica
}

void recibir() {
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    switch (rx) {
      case ESPERA_H1:
        if (b == 0xCC) rx = ESPERA_H2;
        break;
      case ESPERA_H2:
        if (b == 0xDD)      { rx = CARGA; rxN = 0; }
        else if (b != 0xCC) { rx = ESPERA_H1; }
        break;
      case CARGA:
        rxBuf[rxN++] = b;
        if (rxN >= LARGO_CMD - 2) { procesarComando(); rx = ESPERA_H1; }
        break;
    }
  }
}

float leerADC() {
  uint16_t suma = 0;                       // 16 x 1023 = 16368 < 65535
  for (uint8_t i = 0; i < N_ADC; i++) suma += analogRead(PIN_POT);
  return (float)suma / (float)N_ADC;
}

void tick() {
  // 1. Watchdog de comunicacion
  if (estado == MARCHA && (uint32_t)(millis() - t_ultimoCmd) > TIMEOUT_MS) {
    huboTimeout = true;
    irASeguro();
  }
  // 2. Actuador (con rampa)
  rampa();
  // 3. Muestra
  Telemetria p;
  p.h1 = 0xAA;  p.h2 = 0xBB;
  p.seq      = seq_tx++;
  p.t_ms     = millis();
  p.adc      = leerADC();
  p.u_ap     = u_ap;
  p.ack      = ack;
  p.flags    = (uint8_t)((estado == MARCHA ? 0x01 : 0) | (huboTimeout ? 0x02 : 0) |
                         (huboAtraso ? 0x04 : 0) | (huboDescarte ? 0x08 : 0));
  p.version  = VERSION_FW;
  p.crc      = crc16((const uint8_t *)&p + 2, sizeof(Telemetria) - 4);
  // 4. Envio sin bloquear: si el canal esta lleno, se descarta este paquete
  if (Serial.availableForWrite() >= (int)sizeof(Telemetria)) {
    Serial.write((const uint8_t *)&p, sizeof(Telemetria));
    huboAtraso = false;  huboDescarte = false;
  } else {
    huboDescarte = true;
  }
}

void setup() {
  esc.attach(PIN_ESC, 500, 2500);   // limites amplios: writeMicroseconds recorta a [min,max]
  esc.writeMicroseconds(PULSO_0);   // pulso de armado desde el primer instante
  Serial.begin(BAUD);               // NO usar while(!Serial): bloquearia el armado
  analogRead(PIN_POT);              // primera lectura descartada
  t_prox = micros();
}

void loop() {
  recibir();
  uint32_t ahora = micros();
  if ((int32_t)(ahora - t_prox) >= 0) {
    t_prox += PERIODO_US;                          // sin deriva: periodo fijo
    if ((int32_t)(ahora - t_prox) >= 0) {          // se perdio mas de un periodo
      huboAtraso = true;
      t_prox = ahora + PERIODO_US;
    }
    tick();
  }
}
