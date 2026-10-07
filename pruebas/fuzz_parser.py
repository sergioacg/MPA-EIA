import random, struct, importlib.util, sys
spec = importlib.util.spec_from_file_location('pp', 'prueba_protocolo.py')
# reutiliza crc8 y Parse sin correr las pruebas: se copian aqui
src = open('prueba_protocolo.py').read()
ns = {}
exec(src.split("# ---------- helpers del firmware ----------")[0].replace("fw = ctypes.CDLL('./libfw.so')","").replace("fw.pop_tx.restype = ctypes.c_int",""), ns)
crc16, Parse = ns['crc16'], ns['Parse']
def frame(seq, t, adc, u):
    body = struct.pack('<HIffHBB', seq, t, adc, u, seq, 0x01, 2)
    return bytes([0xAA,0xBB]) + body + struct.pack('<H', crc16(body))
tot_bad = tot_new = 0
for seed in range(200):
    rnd = random.Random(seed)
    st = bytearray()
    ref = {}
    for k in range(1500):
        f = frame(k % 65536, 20*k, 100 + (k % 900) + 0.25, 30.0); st += f; ref[k % 65536] = (100 + (k % 900) + 0.25, 20*k/1000)
    s2 = bytearray(st[rnd.randint(0,21):])
    for _ in range(40): del s2[rnd.randint(0, len(s2)-1)]
    for _ in range(40): i = rnd.randint(0, len(s2)-1); s2[i] ^= 1 << rnd.randint(0,7)
    P = Parse()
    for i in range(0, len(s2)-22, 22):
        o = P(bytes(s2[i:i+22]))
        if o['nuevo']:
            tot_new += 1
            r = ref.get(o['seq'])
            if r is None or abs(r[0]-o['adc'])>1e-6 or abs(r[1]-o['t'])>1e-9: tot_bad += 1
print(f'CRC-16: {tot_bad} paquetes corruptos entregados de {tot_new} ({200*80} fallas inyectadas)')
