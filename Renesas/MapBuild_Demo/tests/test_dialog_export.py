import csv, json, tempfile, unittest
from pathlib import Path
from dialogs import FIELDS, get_path, parameters_from_values
from export_data import export_experiment
from interface import validate
from simulation import defaults, Simulation
from run_demo import build

class DialogAndExportTests(unittest.TestCase):
    def values(self,s):return {path:str(get_path(s,path)) for fields in FIELDS.values() for _,path,_,_ in fields}
    def test_parameters_and_invalid(self):
        s=defaults();s['monitors']=[[10,10]];v=self.values(s);v[('odom','scale')]='0.85';result,changed=parameters_from_values(s,v,validate)
        self.assertEqual(result['odom']['scale'],.85);self.assertFalse(changed);self.assertEqual(result['monitors'],[[10,10]])
        v[('map','width')]='90';result,changed=parameters_from_values(s,v,validate);self.assertTrue(changed);self.assertEqual(result['monitors'],[])
        v[('lidar','rays')]='nan'
        with self.assertRaises(ValueError):parameters_from_values(s,v,validate)
    def test_export_historical_scope(self):
        s=defaults();sim=Simulation(s,build())
        for _ in range(20):sim.step()
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as td:
            out=export_experiment(td,s,sim.history,1,[(50,50)],{'trajectory','monitors','map','parameters','lidar'})
            meta=json.loads((out/'导出说明.json').read_text(encoding='utf8'));self.assertEqual(meta['frame'],sim.history[1]['frame'])
            with (out/'trajectory.csv').open(encoding='utf-8-sig') as f:rows=list(csv.reader(f));self.assertEqual(len(rows),3)
            with (out/'monitors.csv').open(encoding='utf-8-sig') as f:rows=list(csv.reader(f));self.assertEqual(len(rows),3)
            second=export_experiment(td,s,sim.history,0,[],{'parameters'});self.assertNotEqual(out,second);self.assertFalse((second/'map.png').exists())
if __name__=='__main__':unittest.main()
