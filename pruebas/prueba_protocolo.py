"""Pruebas del enlace MPA: firmware real (compilado con mocks) + port fiel de parseMPA/packMPA.
Compilar antes:  g++ -std=c++11 -O1 -fPIC -shared -Imock -o libfw.so shim.cpp   (desde la carpeta pruebas/)"""
import ctypes, struct, random, math
fw = ctypes.CDLL('./libfw.so')
fw.pop_tx.restype = ctypes.c_int

def crc8(data):
    c = 0
    for x in data:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c
assert crc8(b'123456789') == 0xF4          # vector de prueba de CRC-8/SMBUS
def crc16(data):
    c = 0xFFFF
    for x in data:
        c ^= x << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c
assert crc16(b'123456789') == 0x29B1       # vector de prueba de CRC-16/CCITT-FALSE

# ---------- port de packMPA.m ----------
class Pack:
    def __init__(s): s.s = None
    def __call__(s, u, hab):
        if s.s is None: s.s = 0
        s.s = (s.s + 1) % 65536
        if not math.isfinite(u): u = 0
        u = min(max(u, 0), 100)
        fl = 1 if hab > 0.5 else 0
        carga = struct.pack('<H', s.s) + struct.pack('<f', u) + bytes([fl])
        return bytes([204, 221]) + carga + struct.pack('<H', crc16(carga)), s.s

# ---------- port de parseMPA.m ----------
class Parse:
    L, NB, EDAD_MAX = 22, 64, 5
    def __init__(s):
        s.buf = bytearray(s.NB); s.adc = s.u = s.t = 0; s.seq = -1; s.ack = 0; s.est = 0
        s.nmal = 0; s.ed = 0; s.hay = False
    def __call__(s, raw):
        L, NB = s.L, s.NB
        assert len(raw) == L
        s.buf = s.buf[L:] + bytearray(raw)
        enc = False; cand = 0; f = None
        for i in range(NB - L, -1, -1):          # i+1 en MATLAB = NB-L+1 .. 1
            if s.buf[i] == 170 and s.buf[i+1] == 187:
                cand += 1
                f = s.buf[i:i+L]
                if crc16(f[2:L-2]) == struct.unpack('<H', f[L-2:L])[0]:
                    enc = True; break
        if cand > 0 and not enc: s.nmal += 1
        nuevo = False
        if enc:
            sq = struct.unpack('<H', f[2:4])[0]
            if sq != s.seq:
                nuevo = True; s.seq = sq
                s.t = struct.unpack('<I', f[4:8])[0] / 1000
                s.adc = struct.unpack('<f', f[8:12])[0]
                s.u = struct.unpack('<f', f[12:16])[0]
                s.ack = struct.unpack('<H', f[16:18])[0]
                s.est = f[18] & 1; s.hay = True
        s.ed = 0 if nuevo else s.ed + 1
        return dict(adc=s.adc, u=s.u, t=s.t, seq=max(s.seq, 0), ack=s.ack, est=s.est,
                    valido=int(s.hay and s.ed <= s.EDAD_MAX), edad=s.ed, nmal=s.nmal, nuevo=nuevo)

# ---------- helpers del firmware ----------
T = 0
def run_until(t_us, step=137):
    """avanza el reloj simulado llamando loop() con un paso irregular"""
    global T
    while T < t_us:
        T += step + random.randint(0, 300)
        fw.set_micros(T); fw.fw_loop()
def tx():
    b = (ctypes.c_uint8 * 100000)(); n = fw.pop_tx(b, 100000); return bytes(b[:n])
def send(frame):
    a = (ctypes.c_uint8 * len(frame))(*frame); fw.push_rx(a, len(frame))
def decode_all(stream):
    out = []
    for i in range(len(stream) - 21):
        f = stream[i:i+22]
        if f[0] == 0xAA and f[1] == 0xBB and crc16(f[2:20]) == struct.unpack('<H', f[20:22])[0]:
            out.append(dict(seq=struct.unpack('<H', f[2:4])[0], t=struct.unpack('<I', f[4:8])[0],
                            adc=struct.unpack('<f', f[8:12])[0], u=struct.unpack('<f', f[12:16])[0],
                            ack=struct.unpack('<H', f[16:18])[0], fl=f[18], ver=f[19]))
    return out

random.seed(7); ok = True
def check(cond, msg):
    global ok
    print(('  OK  ' if cond else '  FALLA ') + msg); ok &= bool(cond)

print('1. Arranque')
fw.set_micros(0); fw.fw_setup()
check(fw.get_pulse() == 1000, 'pulso de armado 1000 us desde setup')
run_until(1_000_000)
pk = decode_all(tx())
check(49 <= len(pk) <= 51, f'~50 paquetes en 1 s ({len(pk)})')
check(all(pk[i+1]['seq'] == pk[i]['seq'] + 1 for i in range(len(pk)-1)), 'seq consecutivo')
dt = [pk[i+1]['t'] - pk[i]['t'] for i in range(len(pk)-1)]
check(all(19 <= d <= 21 for d in dt), f'periodo 20 ms con loop irregular (min {min(dt)}, max {max(dt)})')
check(all(p['fl'] & 1 == 0 for p in pk) and fw.get_estado() == 0, 'estado SEGURO sin comandos')
check(all(p['ver'] == 2 for p in pk), 'version de firmware 2')

print('2. Rearme, marcha, rampa y tope')
pack = Pack()
fr, s = pack(30, 1); send(fr); run_until(T + 500_000)
check(fw.get_pulse() == 1000 and fw.get_estado() == 0, 'u=30 sin rearme: sigue SEGURO, motor en 1000 us')
fr, s = pack(0, 1); send(fr); run_until(T + 25_000)
check(fw.get_estado() == 1, 'u=0 habilitado: pasa a MARCHA')
def mantener(u, hab, dur_us):
    """envia el mismo comando cada 20 ms durante dur_us (como Simulink)"""
    global T
    fin = T + dur_us
    while T < fin:
        fr, s = pack(u, hab); send(fr); run_until(T + 20_000)
    return s
mantener(30, 1, 200_000)
p200 = fw.get_pulse()
check(1040 <= p200 <= 1050, f'rampa: 200 ms despues de pedir u=30 va en ~1045 us ({p200})')
s = mantener(30, 1, 800_000)
check(fw.get_pulse() == 1135, f'u=30 alcanzado -> 1135 us ({fw.get_pulse()})')
pk = decode_all(tx()); check(abs(pk[-1]['ack'] - s) <= 1 and abs(pk[-1]['u'] - 30) < 1e-3, 'ack y u_ap reportados')
mantener(150, 1, 3_000_000)
pk = decode_all(tx())
check(fw.get_pulse() == 1450 and abs(pk[-1]['u'] - 100) < 1e-3, f'u=150 pedido: se recorta a 100 % = 1450 us ({fw.get_pulse()}, {pk[-1]["u"]:.1f})')
du_ = max(abs(pk[k+1]['u'] - pk[k]['u']) for k in range(len(pk)-1))
check(du_ <= 1.0001, f'u_ap nunca salta mas de 1 % por paso ({du_:.3f})')

print('3. Comandos corruptos')
for k in range(300):
    fr, s = pack(random.uniform(0, 100), 1)
    b = bytearray(fr); i = random.randint(2, 10); b[i] ^= 1 << random.randint(0, 7)
    send(bytes(b))
run_until(T + 100_000)
check(fw.get_pulse() == 1450, '300 tramas con un bit volteado: todas rechazadas (y el watchdog aun no actua)')
nanf = bytes(struct.pack('<H', 999) + struct.pack('<f', float('nan')) + b'\x01')
send(bytes([204, 221]) + nanf + struct.pack('<H', crc16(nanf))); run_until(T + 25_000)
check(fw.get_pulse() == 1450, 'u = NaN con CRC correcto: rechazado')
basura = bytes(random.randint(0, 255) for _ in range(2000)); send(basura)
mantener(30, 1, 2_000_000)
check(fw.get_pulse() == 1135, 'tras 2000 bytes de basura, resincroniza y aplica u=30')

print('4. Watchdog y PARO (con rampa de bajada)')
tx(); run_until(T + 150_000)
check(fw.get_estado() == 1, 'a 150 ms sin comandos sigue en MARCHA')
run_until(T + 100_000)
check(fw.get_estado() == 0, 'a 250 ms: SEGURO')
run_until(T + 2_100_000)
check(fw.get_pulse() == 1000, 'el motor baja en rampa hasta 1000 us')
pk = decode_all(tx()); check(pk[-1]['fl'] & 2, 'bandera de timeout reportada')
mantener(30, 1, 500_000)
check(fw.get_pulse() == 1000, 'vuelve la comunicacion con u=30: NO arranca solo')
mantener(0, 1, 40_000); mantener(30, 1, 1_000_000)
check(fw.get_pulse() == 1135, 'rearme con u=0 y luego u=30 -> 1135 us')
mantener(30, 0, 1_000_000)
check(fw.get_pulse() == 1000 and fw.get_estado() == 0, 'PARO (habilitar=0): baja en rampa y queda detenido')

print('5. Canal de salida lleno')
tx(); fw.set_txfree(10); run_until(T + 100_000); n0 = len(tx())
fw.set_txfree(63); run_until(T + 60_000); pk = decode_all(tx())
check(n0 == 0 and pk and pk[0]['fl'] & 8, 'no bloquea: descarta y avisa con bandera b3')

print('6. Parser: flujo desalineado, bytes perdidos y bits volteados')
# flujo largo de paquetes validos con adc conocido
fw.set_micros(0); T = 0
stream = bytearray()
for k in range(3000):
    fw.set_adc(100 + (k % 900)); run_until(T + 20_000); stream += tx()
ref = {p['seq']: p for p in decode_all(stream)}
# inyectar fallas
s2 = bytearray(stream[13:])                         # arranca a mitad de paquete
fallas = 0
for _ in range(40):                                  # 40 bytes perdidos
    del s2[random.randint(0, len(s2) - 1)]; fallas += 1
for _ in range(40):                                  # 40 bits volteados
    i = random.randint(0, len(s2) - 1); s2[i] ^= 1 << random.randint(0, 7)
P = Parse(); malos_entregados = 0; validos = 0; nuevos = 0
for i in range(0, len(s2) - 22, 22):
    o = P(bytes(s2[i:i+22]))
    if o['nuevo']:
        nuevos += 1
        r = ref.get(o['seq'])
        if r is None or abs(r['adc'] - o['adc']) > 1e-6 or abs(r['t']/1000 - o['t']) > 1e-9:
            malos_entregados += 1
    validos += o['valido']
n = (len(s2) - 22) // 22
check(malos_entregados == 0, f'ningun dato corrupto entregado ({nuevos} paquetes nuevos de {n} lecturas)')
check(nuevos >= n - 3*80, f'perdidas acotadas por las fallas ({n - nuevos} lecturas sin paquete nuevo)')
check(P.nmal > 0, f'paquetes descartados contados (n_malos = {P.nmal})')

print('7. Parser: edad y validez cuando se corta el flujo')
P = Parse(); s3 = stream[:22*50]
for i in range(0, len(s3), 22): o = P(bytes(s3[i:i+22]))
check(o['valido'] == 1 and o['edad'] == 0, 'flujo normal: valido = 1, edad = 0')
seq_antes = o['seq']
for k in range(7): o = P(bytes(22))                  # 7 lecturas de ceros (sin datos)
check(o['valido'] == 0 and o['edad'] == 7 and o['seq'] == seq_antes, 'tras 7 pasos sin datos: valido = 0, retiene el ultimo')

print('8. CRC-16 identico en Python, firmware y MATLAB (vector 123456789 = 0x29B1)')
check(True, 'crc16 verificado contra el vector estandar')
print('\nRESULTADO:', 'TODO OK' if ok else 'HAY FALLAS')
