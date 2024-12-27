
import subprocess

vehicle_model = "x500"

command = ["make", "px4_sitl"]

subprocess.run(command, check=True)