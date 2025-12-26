from machine import Pin, SPI, SoftSPI, I2C
from time import sleep, sleep_ms, time
import ujson
import os
import sdcard
from ulora import LoRa, ModemConfig
# ==================== LoRa — WILL NEVER FAIL ====================
lora_spi = SoftSPI(baudrate=5000000, sck=Pin(2), mosi=Pin(3), miso=Pin(4))
lora = LoRa(lora_spi, interrupt=9, this_address=1, cs_pin=5, reset_pin=6,
            freq=915.0, tx_power=20,
            modem_config=ModemConfig.Bw125Cr45Sf128, acks=False)
print(">>> LORA READY — WILL TRANSMIT FOREVER")
# ==================== Try to mount SD (fails silently) ====================
sd_ok = False
try:
    spi1 = SPI(1, baudrate=8000000, sck=Pin(10), mosi=Pin(11), miso=Pin(12))
    sd = sdcard.SDCard(spi1, Pin(13))
    os.mount(os.VfsFat(sd), "/sd")
    with open("/sd/flight_log.csv", "a") as f: f.write("ts,lat,lon,alt,temp,pres\n")
    sd_ok = True
    print(">>> SD CARD OK")
except:
    print(">>> NO SD CARD — STILL TRANSMITTING")
# ==================== MS8607 — fails gracefully ====================
i2c_ok = False
C1 = C2 = C3 = C4 = C5 = C6 = 0
try:
    i2c = I2C(1, sda=Pin(26), scl=Pin(27), freq=100000)
    i2c.writeto(0x76, b'\x1E'); sleep_ms(20)
    C = [int.from_bytes(i2c.readfrom_mem(0x76, 0xA2 + i*2, 2), 'big') for i in range(6)]
    C1, C2, C3, C4, C5, C6 = C
    i2c_ok = True
    print(">>> MS8607 OK")
except:
    print(">>> NO MS8607 — STILL TRANSMITTING")
def read_ms8607():
    if not i2c_ok: return -99.9, 0.0, 0.0
    try:
        i2c.writeto(0x76, b'\x48'); sleep_ms(12)
        D1 = int.from_bytes(i2c.readfrom_mem(0x76, 0x00, 3), 'big')
        i2c.writeto(0x76, b'\x58'); sleep_ms(12)
        D2 = int.from_bytes(i2c.readfrom_mem(0x76, 0x00, 3), 'big')
        dT = D2 - C5 * 256
        TEMP = 2000 + dT * C6 // 8388608
        OFF = C2 * 65536 + (C4 * dT) // 128
        SENS = C1 * 32768 + (C3 * dT) // 256
        P = (D1 * SENS // 2097152 - OFF) // 32768
        temp_c = TEMP / 100.0
        pres_hpa = P / 100.0
        alt_baro = 44330 * (1 - (pres_hpa / 1013.25) ** 0.1903)
        return round(temp_c, 2), round(pres_hpa, 2), round(alt_baro, 1)
    except:
        return -0.0, 0.0, 0.0
# ==================== GPS — fails gracefully ====================
gps_spi = SPI(0, baudrate=1000000, polarity=0, phase=0,
              sck=Pin(18), mosi=Pin(19), miso=Pin(16))
cs = Pin(17, Pin.OUT, value=1)
def get_gps():
    try:
        cs.value(0)
        gps_spi.write(b'\xFF')
        length = gps_spi.read(1)[0]
        if length > 0:
            data = gps_spi.read(length)
            cs.value(1)
            if len(data) >= 100 and data[0] == 0xB5 and data[1] == 0x62 and data[2:4] == b'\x01\x07':
                lat = int.from_bytes(data[28:32], 'little') / 10000000.0
                lon = int.from_bytes(data[32:36], 'little') / 10000000.0
                alt = int.from_bytes(data[36:40], 'little') / 1000.0
                if data[20] >= 2:
                    return round(lat, 6), round(lon, 6), round(alt, 1)
    except:
        pass
    cs.value(1)
    return 0.0, 0.0, 0.0
# ==================== Cutdown ====================
cutdown = Pin(21, Pin.OUT, value=0)
cutdown_triggered = False
# ==================== MAIN LOOP — CANNOT DIE ====================
print(">>> KC1QZM — INDESTRUCTIBLE NAVAL TEST MODE — WILL NEVER STOP")
while True:
    try:
        lat, lon, alt_gps = get_gps()
        temp, pres, alt_baro = read_ms8607()
        final_alt = alt_gps if alt_gps > 10 else alt_baro
        if not cutdown_triggered and final_alt >= 33000:
            print(">>> CUTDOWN FIRED")
            cutdown.value(1); sleep(120); cutdown.value(0)
            cutdown_triggered = True
        payload = ujson.dumps({
            "msg": "KC1YFF",
            "lat": lat,
            "lon": lon,
            "alt": round(final_alt, 1),
            "temp": temp,
            "pres": pres
        }).encode()[:150]
        print(f"TX {len(payload)}B → {lat:.5f},{lon:.5f} {final_alt:.0f}m | {temp}°C {pres}hPa")
        lora.send(payload, 2)  # fire-and-forget — cannot fail
        if sd_ok:
            try:
                with open("/sd/flight_log.csv", "a") as f:
                    f.write(f"{time()},{lat:.6f},{lon:.6f},{final_alt:.1f},{pres:.1f},{temp:.1f}\n")
            except:
                pass  # SD died mid-flight? We don't care — keep transmitting
        sleep(8)
    except Exception as e:
        # Literally nothing can stop this loop
        print("LOOP SURVIVED ERROR:", e)
        sleep(8)