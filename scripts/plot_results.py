import csv
from pathlib import Path
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1]
with open(root/'results/trials.csv',newline='') as f: rows=list(csv.DictReader(f))
errors=[float(r['final_error_m']) for r in rows]
plt.figure()
plt.hist(errors,bins=min(20,max(5,len(errors)//2)))
plt.xlabel('Final alignment error (m)')
plt.ylabel('Trials')
plt.title('Repeated PID simulation trial errors')
plt.tight_layout()
plt.savefig(root/'results/error_distribution.png')
with open(root/'results/trajectory.csv',newline='') as f: trajectory=list(csv.DictReader(f))
plt.figure()
plt.plot([float(r['time_s']) for r in trajectory],[float(r['error_m']) for r in trajectory])
plt.xlabel('Time (s)')
plt.ylabel('Distance from target (m)')
plt.title('Kinematic simulation alignment error')
plt.tight_layout()
plt.savefig(root/'results/error_over_time.png')
print('Saved graphs to results/')
