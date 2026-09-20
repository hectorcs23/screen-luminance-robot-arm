#!/usr/bin/env python3
"""
Captura las lineas CSV que envia el Arduino (VEML7700) y las guarda en disco.
Lo que escribas en la terminal se reenvia al Arduino como etiqueta de brillo.

Requisito:  pip install pyserial
Uso:        python capture.py
"""
import csv
import sys
import threading
import time

import serial

PORT = "COM3"            # Windows: COMx | Linux: /dev/ttyACM0 o /dev/ttyUSB0 | Mac: /dev/cu.usbmodem*
BAUD = 115200
OUTFILE = "luminancia.csv"


def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)  # el Arduino se reinicia al abrir el puerto; esperamos su encabezado

    f = open(OUTFILE, "w", newline="")
    writer = csv.writer(f)

    def reader():
        while True:
            raw = ser.readline().decode(errors="ignore").strip()
            if not raw:
                continue
            if raw.startswith("#"):       # mensajes de estado -> solo a pantalla
                print(raw)
                continue
            print(raw)
            writer.writerow(raw.split(","))
            f.flush()                     # vacia el buffer en cada fila: no se pierden datos

    threading.Thread(target=reader, daemon=True).start()

    print("Escribe la etiqueta de brillo y Enter para disparar una rafaga. Ctrl+C para salir.")
    try:
        for line in sys.stdin:
            ser.write((line.strip() + "\n").encode())
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
        ser.close()
        print(f"\nGuardado en {OUTFILE}")


if __name__ == "__main__":
    main()
