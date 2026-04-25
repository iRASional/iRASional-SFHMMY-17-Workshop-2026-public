import sys
import subprocess
import time

print("Launching Markov Localization UI...", file=sys.stderr)

markov_process = subprocess.Popen("python markov_localization_final.py")

# Αναμονή για να ανοίξει το παράθυρο
time.sleep(2)

# Ρυθμίσεις Φιλτραρίσματος
DETECT_THRESHOLD = 3 
EMPTY_THRESHOLD = 15 
BRIDGE_FILE = "signal.txt"

# Μεταβλητές κατάστασης
detect_counter = 0
empty_counter = 0
current_candidate = None
is_locked = False

VALID_SIGNS = ["stop", "priority", "crosswalk", "highway", "parking", "roundabout"]

print("Filter Script Started. Waiting for YOLO data...", file=sys.stderr)

try:
    for line in sys.stdin:
        line = line.lower().strip()
        if not line: continue

        found_sign = None
        for sign in VALID_SIGNS:
            if sign in line:
                found_sign = sign
                break

        if found_sign:
            empty_counter = 0 # Βρήκαμε κάτι, μηδένισε την αναμονή κενού
            
            if found_sign == current_candidate:
                detect_counter += 1
            else:
                current_candidate = found_sign
                detect_counter = 1
            
        

            # Έλεγχος αν πρέπει να στείλουμε το σήμα
            if detect_counter >= DETECT_THRESHOLD and not is_locked:
                with open(BRIDGE_FILE, "w") as f:
                    f.write(found_sign)
                is_locked = True
                
        else:
            # Αν η γραμμή δεν περιέχει σήμα (π.χ. "no detections")
            empty_counter += 1
            if empty_counter >= EMPTY_THRESHOLD:
                if is_locked:
                    print(">>> RESET: Ready for next sign", file=sys.stderr)
                is_locked = False
                detect_counter = 0
                current_candidate = None

except KeyboardInterrupt:
    print("\nShutting down...", file=sys.stderr)
finally:
    markov_process.terminate()