import subprocess
import os

class HydroSP:
    def __init__(self, binary_path, device_path):
        
        self.binary_path = os.path.abspath(binary_path)
        self.device_path = device_path
        
        if not os.path.exists(self.binary_path):
            raise FileNotFoundError(f"Утилита hydrosp из serial-protocol-desktop не найдена по пути: {self.binary_path}")

    def _run_command(self, cmd_args: list) -> str:

        full_command = [self.binary_path] + cmd_args
        
        result = subprocess.run(
            full_command, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE, 
            text=True,
            check=True
        )
        return result.stdout.strip()

    def write(self, slave: int, reg: int, value: str, as_integer: bool = False) -> str:
        """
        hydrosp write -s <slave> -r <reg> [-i] -d <device> <value>
        """
        args = ["write", "-s", str(slave), "-r", str(reg)]
        if as_integer:
            args.append("-i")
        args.extend(["-d", self.device_path, str(value)])
        
        return self._run_command(args)

    def read(self, slave: int, reg: int, length: int = 1, as_integer: bool = False) -> str:
        """
        hydrosp read -s <slave> -r <reg> [-i] [-l <length>] -d <device>
        """
        args = ["read", "-s", str(slave), "-r", str(reg)]
        if as_integer:
            args.append("-i")
        if length != 1:
            args.extend(["-l", str(length)])
        args.extend(["-d", self.device_path])
        
        return self._run_command(args)
