"""
maquina.py
==========
Clase Python para comunicarse con el ESP32 vía Modbus TCP.

Instalación:
    pip install pymodbus

Uso rápido:
    from maquina import Maquina
    m = Maquina("192.168.10.2")

    m.luminaria1(True)          # encender luminaria 1
    m.luminaria2(False)         # apagar luminaria 2
    m.cinta_adelante()          # activar modo auto-forward (con fotocélula)
    m.cinta_atras()             # marcha atrás directa
    m.cinta_parar()             # parar motor
    m.set_velocidad(180)        # cambiar velocidad 0-255

    estado = m.estado_motor()   # "parado" | "forward" | "backward"
    dins = m.leer_todas_dins()  # dict {"DIN1": True, "DIN2": False, ...}

    # Fotocélula: lectura puntual
    activa = m.fotocelula()     # True si objeto detectado

    # Fotocélula: monitorización continua en background (callback)
    def on_foto(estado: bool):
        print("Fotocélula ->", "ACTIVADA" if estado else "libre")

    m.iniciar_monitor_fotocelula(on_foto, intervalo_ms=100)
    # ... haz otras cosas ...
    m.detener_monitor_fotocelula()

    m.close()
"""

import threading
import time
from typing import Callable, Optional

from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException


# ──────────────────────────────────────────────────────────
#  Mapa de registros  (debe coincidir con esp32_modbus.cpp)
# ──────────────────────────────────────────────────────────
COIL_LUM1       = 0    # Luminaria 1      (R/W)
COIL_LUM2       = 1    # Luminaria 2      (R/W)
COIL_FAWD        = 2    # Cinta forward-Auto    (W)
COIL_BWD        = 3    # Cinta backward   (W)
COIL_FWD        = 5    # Cinta forward  (W)
COIL_STOP       = 4    # Cinta stop       (W)

DINP_BASE       = 0    # DIN1..DIN8 discrete inputs (R)

HREG_SPEED      = 0    # Velocidad motor 0-255       (R/W)

IREG_MOTOR_ST   = 0    # Estado motor 0=parado 1=fwd 2=bwd (R)
IREG_DIN_BYTE   = 1    # Todas las DIN en un byte         (R)

MODBUS_PORT     = 502
UNIT_ID         = 1    # ID de esclavo (el ESP32 usa 1 por defecto)


class Maquina:
    """
    Interfaz Modbus TCP para el ESP32-S3-POE.

    Parámetros
    ----------
    ip : str
        Dirección IP del ESP32 (p.ej. "192.168.1.100").
    port : int
        Puerto Modbus TCP (por defecto 502).
    timeout : float
        Tiempo máximo de espera por respuesta en segundos.
    """

    def __init__(self, ip: str, port: int = MODBUS_PORT, timeout: float = 3.0):
        self._ip      = ip
        self._port    = port
        self._timeout = timeout

        self._client  = ModbusTcpClient(ip, port=port, timeout=timeout)
        self._lock    = threading.Lock()   # protege el cliente en contextos multihilo

        # Variables para el monitor de fotocélula
        self._foto_thread:    Optional[threading.Thread] = None
        self._foto_stop_evt:  threading.Event = threading.Event()
        self._foto_callback:  Optional[Callable[[bool], None]] = None
        self._foto_intervalo: float = 0.1   # segundos

        if not self._client.connect():
            raise ConnectionError(f"No se pudo conectar al ESP32 en {ip}:{port}")

    # ──────────────────────────────────────────────────────
    #  Conexión / cierre
    # ──────────────────────────────────────────────────────

    def reconnect(self) -> bool:
        """Intenta reconectar si se perdió la conexión."""
        with self._lock:
            self._client.close()
            return self._client.connect()

    def close(self):
        """Cierra la conexión Modbus. Detiene el monitor si estuviera activo."""
        self.detener_monitor_fotocelula()
        with self._lock:
            self._client.close()

    # ──────────────────────────────────────────────────────
    #  Helpers internos
    # ──────────────────────────────────────────────────────

    def _write_coil(self, address: int, value: bool) -> bool:
        """Escribe un coil. Devuelve True si OK."""
        with self._lock:
            try:
                result = self._client.write_coil(address, value, device_id=UNIT_ID)
                return not result.isError()
            except ModbusException as exc:
                print(f"[Modbus] Error escribiendo coil {address}: {exc}")
                return False

    def _read_coil(self, address: int) -> Optional[bool]:
        """Lee un coil. Devuelve True/False o None si hay error."""
        with self._lock:
            try:
                result = self._client.read_coils(address, count=1, device_id=UNIT_ID)
                if result.isError():
                    return None
                return bool(result.bits[0])
            except ModbusException as exc:
                print(f"[Modbus] Error leyendo coil {address}: {exc}")
                return None

    def _read_discrete(self, address: int) -> Optional[bool]:
        """Lee una entrada discreta (DIN). Devuelve True/False o None."""
        with self._lock:
            try:
                result = self._client.read_discrete_inputs(address, count=1, device_id=UNIT_ID)
                if result.isError():
                    return None
                return bool(result.bits[0])
            except ModbusException as exc:
                print(f"[Modbus] Error leyendo discrete input {address}: {exc}")
                return None

    def _read_discretes_all(self) -> Optional[list[bool]]:
        """Lee las 8 entradas discretas de una vez."""
        with self._lock:
            try:
                result = self._client.read_discrete_inputs(DINP_BASE, count=8, device_id=UNIT_ID)
                if result.isError():
                    return None
                return list(result.bits[:8])
            except ModbusException as exc:
                print(f"[Modbus] Error leyendo entradas discretas: {exc}")
                return None

    def _write_hreg(self, address: int, value: int) -> bool:
        """Escribe un holding register."""
        with self._lock:
            try:
                result = self._client.write_register(address, value, device_id=UNIT_ID)
                return not result.isError()
            except ModbusException as exc:
                print(f"[Modbus] Error escribiendo hreg {address}: {exc}")
                return False

    def _read_hreg(self, address: int) -> Optional[int]:
        """Lee un holding register."""
        with self._lock:
            try:
                result = self._client.read_holding_registers(address, count=1, device_id=UNIT_ID)
                if result.isError():
                    return None
                return result.registers[0]
            except ModbusException as exc:
                print(f"[Modbus] Error leyendo hreg {address}: {exc}")
                return None

    def _read_ireg(self, address: int) -> Optional[int]:
        """Lee un input register."""
        with self._lock:
            try:
                result = self._client.read_input_registers(address, count=1, device_id=UNIT_ID)
                if result.isError():
                    return None
                return result.registers[0]
            except ModbusException as exc:
                print(f"[Modbus] Error leyendo ireg {address}: {exc}")
                return None

    # ──────────────────────────────────────────────────────
    #  LUMINARIAS
    # ──────────────────────────────────────────────────────

    def luminaria1(self, encender: bool) -> bool:
        """
        Controla la luminaria 1.

        Parámetros
        ----------
        encender : bool
            True → encender, False → apagar.

        Devuelve True si la orden se envió correctamente.
        """
        return self._write_coil(COIL_LUM1, encender)

    def luminaria2(self, encender: bool) -> bool:
        """
        Controla la luminaria 2.

        Parámetros
        ----------
        encender : bool
            True → encender, False → apagar.

        Devuelve True si la orden se envió correctamente.
        """
        return self._write_coil(COIL_LUM2, encender)

    def estado_luminaria1(self) -> Optional[bool]:
        """Lee el estado actual de la luminaria 1 (True=encendida)."""
        return self._read_coil(COIL_LUM1)

    def estado_luminaria2(self) -> Optional[bool]:
        """Lee el estado actual de la luminaria 2 (True=encendida)."""
        return self._read_coil(COIL_LUM2)

    # ──────────────────────────────────────────────────────
    #  CINTA / MOTOR
    # ──────────────────────────────────────────────────────

    def cinta_adelante_auto(self) -> bool:
        """
        Activa el modo auto-forward de la cinta.
        El ESP32 controlará el arranque/paro según la fotocélula (DIN1).

        Devuelve True si la orden se envió correctamente.
        """
        return self._write_coil(COIL_FAWD, True)
    
    def cinta_adelante(self) -> bool:
        """
        Activa marcha adelante directa del motor (sin fotocélula).

        Devuelve True si la orden se envió correctamente.
        """
        return self._write_coil(COIL_FWD, True)

    def cinta_atras(self) -> bool:
        """
        Activa marcha atrás directa del motor (sin fotocélula).

        Devuelve True si la orden se envió correctamente.
        """
        return self._write_coil(COIL_BWD, True)

    def cinta_parar(self) -> bool:
        """
        Para el motor de la cinta.

        Devuelve True si la orden se envió correctamente.
        """
        return self._write_coil(COIL_STOP, True)

    def set_velocidad(self, velocidad: int) -> bool:
        """
        Establece la velocidad del motor (0-255).
        Si el motor está en marcha, el cambio es inmediato.

        Parámetros
        ----------
        velocidad : int  (0-255)

        Devuelve True si la orden se envió correctamente.
        """
        velocidad = max(0, min(255, int(velocidad)))
        return self._write_hreg(HREG_SPEED, velocidad)

    def get_velocidad(self) -> Optional[int]:
        """
        Lee la velocidad configurada actualmente en el ESP32 (0-255).
        """
        return self._read_hreg(HREG_SPEED)

    def estado_motor(self) -> Optional[str]:
        """
        Lee el estado del motor.

        Devuelve
        --------
        "parado"   → motor detenido
        "forward"  → girando adelante
        "backward" → girando atrás
        None       → error de comunicación
        """
        val = self._read_ireg(IREG_MOTOR_ST)
        if val is None:
            return None
        return {0: "parado", 1: "forward", 2: "backward"}.get(val, "desconocido")

    # ──────────────────────────────────────────────────────
    #  ENTRADAS DIGITALES (DIN)
    # ──────────────────────────────────────────────────────

    def fotocelula(self) -> Optional[bool]:
        """
        Lee el estado puntual de la fotocélula (DIN1).

        Devuelve
        --------
        True  → objeto detectado (señal ALTA)
        False → sin objeto (señal BAJA)
        None  → error de comunicación
        """
        return self._read_discrete(DINP_BASE + 0)   # DIN1 = bit 0

    def leer_din(self, canal: int) -> Optional[bool]:
        """
        Lee una entrada digital concreta.

        Parámetros
        ----------
        canal : int  (1-8)

        Devuelve True/False o None si hay error.
        """
        if canal < 1 or canal > 8:
            raise ValueError("El canal DIN debe estar entre 1 y 8.")
        return self._read_discrete(DINP_BASE + canal - 1)

    def leer_todas_dins(self) -> Optional[dict[str, bool]]:
        """
        Lee las 8 entradas digitales de una sola petición Modbus.

        Devuelve un diccionario  {"DIN1": bool, "DIN2": bool, ..., "DIN8": bool}
        o None si hay error de comunicación.
        """
        bits = self._read_discretes_all()
        if bits is None:
            return None
        return {f"DIN{i+1}": bits[i] for i in range(8)}

    # ──────────────────────────────────────────────────────
    #  FOTOCÉLULA  —  Monitor en background (hilo)
    # ──────────────────────────────────────────────────────

    def iniciar_monitor_fotocelula(
        self,
        callback: Callable[[bool], None],
        intervalo_ms: int = 100,
    ):
        """
        Lanza un hilo que lee la fotocélula cada `intervalo_ms` ms
        y llama a `callback(estado: bool)` cada vez que el estado cambia.

        Parámetros
        ----------
        callback : Callable[[bool], None]
            Función que se invoca al cambio de estado.
            Recibe True (objeto detectado) o False (libre).
        intervalo_ms : int
            Período de muestreo en milisegundos (por defecto 100 ms = 10 Hz).

        Ejemplo
        -------
            def on_foto(activa: bool):
                print("Fotocélula:", "DETECTA" if activa else "libre")

            m.iniciar_monitor_fotocelula(on_foto, intervalo_ms=50)
        """
        if self._foto_thread and self._foto_thread.is_alive():
            print("[Monitor] El monitor de fotocélula ya está activo.")
            return

        self._foto_callback  = callback
        self._foto_intervalo = intervalo_ms / 1000.0
        self._foto_stop_evt.clear()

        self._foto_thread = threading.Thread(
            target=self._foto_worker,
            daemon=True,
            name="foto-monitor",
        )
        self._foto_thread.start()
        print(f"[Monitor] Monitor de fotocélula iniciado ({intervalo_ms} ms).")

    def detener_monitor_fotocelula(self):
        """Detiene el hilo de monitorización de la fotocélula."""
        if self._foto_thread and self._foto_thread.is_alive():
            self._foto_stop_evt.set()
            self._foto_thread.join(timeout=2.0)
            print("[Monitor] Monitor de fotocélula detenido.")
        self._foto_thread = None

    def _foto_worker(self):
        """
        Bucle interno del hilo de fotocélula.
        Solo llama al callback cuando el estado CAMBIA (flanco).
        """
        estado_anterior: Optional[bool] = None

        while not self._foto_stop_evt.is_set():
            estado_actual = self.fotocelula()

            if estado_actual is not None and estado_actual != estado_anterior:
                estado_anterior = estado_actual
                if self._foto_callback:
                    try:
                        self._foto_callback(estado_actual)
                    except Exception as exc:
                        print(f"[Monitor] Error en callback fotocélula: {exc}")

            self._foto_stop_evt.wait(self._foto_intervalo)

    # ──────────────────────────────────────────────────────
    #  Context manager  (with Maquina(...) as m:)
    # ──────────────────────────────────────────────────────

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def __repr__(self):
        return f"Maquina(ip={self._ip!r}, port={self._port})"


# # ──────────────────────────────────────────────────────────
# #  Demo rápida
# # ──────────────────────────────────────────────────────────
# if __name__ == "__main__":

#     IP_ESP32 = "192.168.10.2"   # ← cambia por la IP de tu ESP32

#     def on_fotocelula(activa: bool):
#         if activa:
#             print("  🔴 Fotocélula ACTIVADA  (objeto detectado)")
#         else:
#             print("  🟢 Fotocélula libre")

#     print(f"Conectando a {IP_ESP32} ...")

#     with Maquina(IP_ESP32) as m:
#         print("Conectado.\n")

#         # --- Luminarias ---
#         print("Encendiendo luminaria 1...")
#         m.luminaria1(True)
#         time.sleep(1)

#         print("Encendiendo luminaria 1...")
#         m.luminaria2(True)
#         time.sleep(1)

#         # print("Apagando luminaria 1...")
#         # m.luminaria1(False)
#         # time.sleep(0.5)

#         # --- Velocidad y cinta ---
#         print("Velocidad → 180")
#         m.set_velocidad(180)

#         print("Cinta ADELANTE (modo auto con fotocélula)...")
#         m.cinta_adelante()

        
#         # --- Monitor fotocélula en background ---
#         #print("\nIniciando monitor de fotocélula (5 s)...")
#         m.iniciar_monitor_fotocelula(on_fotocelula, intervalo_ms=100)

#         # Lectura directa de todas las DINs
#         dins = m.leer_todas_dins()
#         print(f"\nEstado DINs: {dins}")

#         print(f"Estado motor: {m.estado_motor()}")
#         print(f"Velocidad configurada: {m.get_velocidad()}")
    
#         time.sleep(10)   # deja correr el monitor 5 segundos

#         # --- Parar todo ---
#         print("\nParando cinta y apagando todo...")
#         m.cinta_parar()
#         m.luminaria1(False)
#         m.luminaria2(False)

#     print("Conexión cerrada.")
