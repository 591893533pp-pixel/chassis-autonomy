"""Summarize completed suspended-wheel tests; no CAN access."""
import csv,json,statistics,sys
from pathlib import Path
folder=Path(sys.argv[1]); summary=[]
steady_end=float(sys.argv[2]) if len(sys.argv)>2 else 4.9
for p in sorted(folder.glob('id*-*.csv')):
 rows=list(csv.DictReader(p.open()))
 if not rows:continue
 i=int(rows[0]['selected_id']);steady=[r for r in rows if 1<=float(r['elapsed_s'])<steady_end]
 if not steady:continue
 target=float(steady[-1]['target_rad_s']);values=[float(r[f'wheel{i}_rad_s']) for r in steady]
 summary.append(dict(file=p.name,id=i,target=target,n=len(steady),mean=statistics.mean(values),std=statistics.pstdev(values),error_percent=100*(statistics.mean(values)-target)/abs(target),zero_target_std={str(j):statistics.pstdev(float(r[f'wheel{j}_rad_s']) for r in steady) for j in range(1,5) if j!=i},max_age_ms=max(float(r['age_ms']) for r in rows),invalid=sum(r['valid']!='1' for r in rows),faults={str(j):sorted(set(int(r[f'wheel{j}_fault']) for r in rows)) for j in range(1,5)},final_target=float(rows[-1]['target_rad_s'])))
(folder/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
