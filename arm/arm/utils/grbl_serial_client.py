import serial
import time

class GrblSerialClient:
    def __init__(self, port, baud_rate, timeout):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.puerto_serial = None

    def connect(self) -> tuple[bool, str]:
        aux = None
        try:
            self.puerto_serial = serial.Serial(port=self.port, baudrate=self.baud_rate, timeout=self.timeout)
            if not self.puerto_serial.is_open:
                raise ConnectionError("No se pudo abrir el puerto serie")
            
            startup_deadline = time.monotonic() + 5.0
            while time.monotonic() < startup_deadline:
                raw_response = self.puerto_serial.readline()
                response =raw_response.decode("utf-8", "replace").strip()

                if response.startswith("Grbl"):
                    print(f"GRBL inicializado correctamente {response}")
                    return True, response
            
            self.disconnect()
            raise TimeoutError("El puerto se abrió, pero GRBL no termino la inicialización.")

        except serial.SerialException as error:
            raise ConnectionError(
                f"Error de comunicación serie: {error}"
            ) from error

    def disconnect(self) -> tuple[bool, str]:
        try:
            if self.puerto_serial is not None and self.puerto_serial.is_open:
                self.puerto_serial.close()
                return True, 'Puerto cerrado.'

        except serial.SerialException as e:
            print(f'Error al cerrar el puerto {e}.')


    def send_line_command(self, command, command_timeout=240.0):
        """
        Escribe el comando y espera el 'ok'
        """
        if self.puerto_serial is None or not self.puerto_serial.is_open:
            raise ConnectionError("El puerto serie del Arm no está conectado.")
        
        responses = []
        try:
            command_bytes = f"{command.strip()}\n".encode("utf-8")
            self.puerto_serial.write(command_bytes)

            startup_deadline = time.monotonic() + command_timeout
            while time.monotonic() < startup_deadline:
                raw_response = self.puerto_serial.readline()
                response = raw_response.decode("utf-8", "replace").strip()
                if response == "":
                    continue

                #print(f"GRBL: {response}")
                
                if response == "ok":
                    responses.append(response)
                    return responses
                
                if response.startswith("error:"):
                    raise RuntimeError(f"GRBL rechazó el comando : {response}")
                
                if response.startswith("ALARM:"):
                    raise RuntimeError(f"GRBL está en alarma: {response}")
                
                responses.append(response)
            raise TimeoutError(
                f"GRBL no completó el comando dentro del tiempo: {command}"
            )

        except serial.SerialException as error:
            print(f'Error al mandar el comando {error}.')

    def write_line_command(self, command: str) -> None:
        """
        Escribe el comando y no es pera el 'ok'
        """
        if self.puerto_serial is None or not self.puerto_serial.is_open:
            raise ConnectionError(
                "El puerto serie del brazo no está conectado."
            )

        try:
            command_bytes = f"{command.strip()}\n".encode("utf-8")
            self.puerto_serial.write(command_bytes)

        except serial.SerialException as error:
            raise ConnectionError(
                f"Error escribiendo comando GRBL: {error}"
            ) from error

    def query_status(self):
        if self.puerto_serial is None or not self.puerto_serial.is_open:
            raise ConnectionError(
                "El puerto serie del brazo no está conectado."
            )

        try:
            self.puerto_serial.write(b"?")
            deadline = time.monotonic() + 15.0

            while time.monotonic() < deadline:
                raw_response = self.puerto_serial.readline()
                response = raw_response.decode(
                    "utf-8",
                    errors="replace",
                ).strip()

                if response == "":
                    continue

                if response.startswith("error:"):
                    return False, {
                        "raw": response,
                        "message": response,
                    }

                if response.startswith("ALARM:"):
                    return False, {
                        "raw": response,
                        "message": response,
                    }

                if response.startswith("<") and response.endswith(">"):
                    parsed = self.parse_status_report(response)
                    return True, parsed

            raise TimeoutError("Timeout consultando estado GRBL")

        except serial.SerialException as error:
            raise ConnectionError(
                f"Error de comunicación serie: {error}"
            ) from error
        
    def parse_status_report(self, status_report: str) -> dict:
        parsed = {
            "raw": status_report,
        }

        content = status_report[1:-1]
        fields = content.split("|")

        machine_state = fields[0]
        parsed["status"] = machine_state
        parsed["base_status"] = machine_state.split(":", 1)[0]

        for field in fields[1:]:
            if ":" not in field:
                continue

            key, value = field.split(":", 1)

            if key in ["MPos", "WPos", "WCO"]:
                parsed[key] = [
                    float(number)
                    for number in value.split(",")
                ]

            elif key == "Bf":
                parsed[key] = [
                    int(number)
                    for number in value.split(",")
                ]

            elif key == "FS":
                parsed[key] = [
                    float(number)
                    for number in value.split(",")
                ]

            elif key == "Pn":
                parsed[key] = value

            else:
                parsed[key] = value

        return parsed
        
    def wait_until_idle(
        self,
        timeout: float,
        poll_period: float = 0.1,
    ) -> str:
        if self.puerto_serial is None or not self.puerto_serial.is_open:
            raise ConnectionError(
                "El puerto serie del brazo no está conectado."
            )

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            _, response = self.query_status()

            base_state = response.get('base_status') 

            if base_state == "Idle":
                return response

            if base_state == "Alarm":
                raise RuntimeError(
                    f"GRBL entró en alarma: {response}"
                )

            if base_state in {"Hold", "Door"}:
                raise RuntimeError(
                    f"Movimiento interrumpido. Estado: {response}"
                )

            time.sleep(poll_period)

        raise TimeoutError(
            f"GRBL no alcanzó Idle en {timeout} segundos"
        )

if __name__ == "__main__":
    SerialClient = GrblSerialClient(port="/dev/ttyACM0", baud_rate=115200, timeout=3)
    SerialClient.connect()
    responses = SerialClient.send_line_command(command='$H')
    print(responses)
    _, response = SerialClient.query_status()
    print(response, response.get('base_status'))
    SerialClient.send_line_command("G21")
    SerialClient.send_line_command("G91")
    # SerialClient.send_line_command(
    #      "G1 X12 F100"
    # )
    SerialClient.send_line_command(
         "G1 X7.0 Y128.0 Z43.0 B157.0 F100"
    )
    response = SerialClient.wait_until_idle(timeout=200.0)
    print(response)
    SerialClient.disconnect()
