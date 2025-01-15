import sys

for line in sys.stdin:
    if line.startswith("v  "):
        parts = line.split()
        if len(parts) >= 3:
            try:
                x = (float(parts[1]) + 1) * 400 / 2
                y = (float(parts[2]) + 1) * 400 / 2 - 200
                z = (float(parts[3]) + 1) * 400 / 2
                print(f"v  {x:.1f} {y:.1f} {z:.1f}")
            except ValueError:
                print(line.strip())
        else:
            print(line.strip())
    else:
        print(line.strip())