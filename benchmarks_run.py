import subprocess
import os

# Define the command you want to run
command_template = "./build/ECE666/gem5.opt --stats-file={}_4May_2M_NoAssoc_Estats.txt configs/splash2/run.py -b {} -n 32 --link-latency=100 --l1i_size=2MB --l1d_size=2MB --l2_size=16MB --cacheline_size=32"

# List of arguments
#arguments = [ "Cholesky", "FFT", "Radix", "WaterNSquared", "FMM", "WaterSpatial" ,"Barnes","OceanNoncontig", "OceanContig", "LUContig", "LUNoncontig"]  # Replace with your desired arguments
arguments = [ "Cholesky", "FFT", "Radix", "WaterNSquared", "OceanNoncontig", "OceanContig", "LUContig", "LUNoncontig"]  # Replace with your desired arguments
#arguments = ["Radix", "WaterNSquared", "OceanContig", "LUContig"]  # Replace with your desired arguments
#arguments = [ "Cholesky", "FFT", "OceanNoncontig", "LUNoncontig"]  # Replace with your desired arguments
#arguments = [ "OceanNoncontig", "OceanContig", "LUContig", "LUNoncontig"]  # Replace with your desired arguments

# Specify the log file path
log_file_path = "run_log.txt"

# Initialize the log file
with open(log_file_path, "w") as log_file:
    log_file.write("Failing commands:\n")

# Run the command for each argument
for arg in arguments:
    current_command = command_template.format(arg, arg)  # Pass the argument twice for the stats file name

    try:
        # Execute the command
        completed_process = subprocess.run(current_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)

        # Capture the last 2 lines of stdout
        stdout_lines = completed_process.stdout.strip().splitlines()
        last_two_stdout_lines = "\n".join(stdout_lines[-2:])

        # Write the last 2 lines of stdout to the log file
        with open(log_file_path, "a") as log_file:
            log_file.write(f"Last 2 lines of stdout for argument {arg}:\n{last_two_stdout_lines}\n")

    except Exception as e:
        # Handle any exceptions (e.g., command not found, invalid arguments, etc.)
        print(f"Error executing command for argument {arg}: {e}")

print("Script execution complete. Check the error log for details.")
