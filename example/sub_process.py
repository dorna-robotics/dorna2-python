import subprocess
import time

process = subprocess.Popen('python ws_test.py',stdout=subprocess.PIPE, stderr=subprocess.PIPE)
while True:
    output = process.stdout.readline()
    """
    error = process.stderr.readline()
    if error:
         print(str(error.strip(), "utf-8"))
    """
    if process.poll() is not None and output == b'':
        break
    if output:
        print(str(output.strip(), "utf-8"))
    time.sleep(0.001)
    process.kill()
retval = process.poll()
