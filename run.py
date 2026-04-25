import subprocess
import sys
import os

# 1. Εντοπισμός της Python
python_exe = sys.executable

# 2. Ορισμός των εντολών
yolo_cmd = [python_exe, "-u", "Yolo_vs.py"]
filter_cmd = [python_exe, "-u", "filter_signals.py"]

print(">>> Starting Workshop System (Cross-Platform Ready)...")

# 3. Ξεκινάμε το YOLO
yolo_proc = subprocess.Popen(
    yolo_cmd,
    stdout=subprocess.PIPE,  # Στέλνουμε την έξοδο στην Python
    text=True,
    bufsize=1 # Line buffering
)

# 4. Ξεκινάμε το Φίλτρο (το οποίο με τη σειρά του θα ανοίξει τον Markov)
# Το stdin του φίλτρου είναι το stdout του YOLO
filter_proc = subprocess.Popen(
    filter_cmd,
    stdin=yolo_proc.stdout,
    text=True,
    bufsize=1
)

print(">>> System is running. Close the camera window to exit.")

try:
    # Περιμένουμε μέχρι να κλείσει το φίλτρο ή το YOLO
    filter_proc.wait()
except KeyboardInterrupt:
    print("\n>>> Manual shutdown initiated...")
finally:
    yolo_proc.terminate()
    filter_proc.terminate()