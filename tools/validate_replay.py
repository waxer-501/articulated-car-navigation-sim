"""Replay a complete real observation log twice, with conflicting score-side truth."""
from pathlib import Path
import json
import shutil
from artic_nav.config import ROOT,load_config
from artic_nav.benchmark import replay

def main():
 source=ROOT/'runs/acceptance_final/straight_seed0'
 target=ROOT/'runs/replay_validation'
 target.mkdir(exist_ok=True)
 shutil.copy2(source/'sensor_requests.jsonl.gz', target/'sensor_requests.jsonl.gz')
 truth=target/'truth_evaluation_only.jsonl'
 shutil.copy2(source/'truth_evaluation_only.jsonl', truth)
 cfg=load_config(source/'config.yaml')
 before=replay(cfg,target/'sensor_requests.jsonl.gz')
 rows=[json.loads(s) for s in truth.read_text().splitlines()]
 for row in rows: row['state']=[999.,-999.,2.5,-.65]
 truth.write_text('\n'.join(json.dumps(r) for r in rows)+'\n')
 after=replay(cfg,target/'sensor_requests.jsonl.gz')
 passed=before['navigation_sha256']==after['navigation_sha256']
 result={'passed':passed,'messages':before['messages'],'before_sha256':before['navigation_sha256'],
         'after_sha256':after['navigation_sha256'],'modified_truth_rows':len(rows)}
 (target/'result.json').write_text(json.dumps(result,indent=2))
 print(json.dumps(result,indent=2))
 assert passed
if __name__=='__main__':main()
