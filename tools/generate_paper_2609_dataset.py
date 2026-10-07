#!/usr/bin/env python3
"""Generate paper-sized instances; original sampling equivalence is unverified."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml
from scipy.ndimage import label

MAPS = {
    'random_32_32': 'random-32-32-10',
    'symbotic_39_37': 'symbotic',
    'den312d_65_81': 'den312d',
    'maze_32_32_2': 'maze-32-32-2',
    'room_64_64_8': 'room-64-64-8',
    'warehouse_161_63': 'warehouse-10-20-10-2-1',
    'orz900d': 'orz900d',
    'Boston': 'Boston_0_256',
}

def generate(root):
    if (root / 'manifest.json').exists():
        raise FileExistsError(f'{root} already contains a dataset; choose a new output directory')
    root.mkdir(parents=True, exist_ok=True)
    repo = Path(__file__).resolve().parents[1]
    cases = []
    for name, stem in MAPS.items():
        source = repo / 'third_party/ITA-CBS2/map_file' / (stem + '.map')
        if stem == 'symbotic':
            source = repo / 'tests/assets/symbotic.map'
        lines = source.read_text().splitlines()
        grid = np.array([[c not in '@T' for c in row] for row in lines[4:]])
        labels, _ = label(grid)
        counts = np.bincount(labels.ravel())
        counts[0] = 0
        points = np.argwhere(labels == counts.argmax())
        # All starts and goals use the largest component. Unique private goals
        # give an explicit perfect-matching witness in the Common Target Test.
        vertex_ids = -np.ones(grid.shape, dtype=np.int64)
        vertex_ids[grid] = np.arange(int(grid.sum()))
        settings = [('group', None)] + [('common', k) for k in (0, 4, 9, 15)]
        for suite, shared in settings:
            ratio = {0: '000', 4: '030', 9: '060', 15: '100'}.get(shared)
            dirname = (f'Paper_{name}_gp_5' if suite == 'group'
                       else f'paper_{name}_ratio_{ratio}')
            folder = root / dirname
            folder.mkdir(exist_ok=True)
            map_path = folder / source.name
            # All three solvers must see the same traversability. ITA-CBS only
            # accepts '.', whereas LaCAM treats every non-@/T character as free.
            normalized = lines[:4] + [''.join(c if c in '@T' else '.' for c in row)
                                      for row in lines[4:]]
            map_path.write_text('\n'.join(normalized) + '\n')
            agent_counts = range(10, 201, 10) if suite == 'group' else range(5 if stem.startswith('maze') else 15, 61, 5)
            for n in agent_counts:
                for seed in range(20):
                    rng = np.random.RandomState(seed)
                    starts = rng.choice(len(points), n, replace=False)
                    if suite == 'group':
                        goals = rng.choice(len(points), n, replace=False)
                        allowed = [goals[i // 5 * 5:i // 5 * 5 + 5].tolist() for i in range(n)]
                        witness = goals.tolist()
                    else:
                        # Private goals are globally distinct and excluded from
                        # all additional targets; additional local sets may overlap.
                        selected = rng.choice(len(points), n + shared, replace=False)
                        private, common = selected[:n], selected[n:]
                        pool = np.setdiff1d(np.arange(len(points)), selected)
                        allowed = [[int(private[i])] + common.tolist() + rng.choice(pool, 15 - shared, replace=False).tolist() for i in range(n)]
                        witness = private.tolist()
                    assert len(set(witness)) == n
                    assert all(witness[i] in allowed[i] for i in range(n))
                    data = {'map': source.name, 'agents': [
                        {'name': f'agent{i}', 'start': points[s].tolist(),
                         'potentialGoals': points[allowed[i]].tolist()}
                        for i, s in enumerate(starts)]}
                    path = folder / f'{stem}_agents_{n}_test_{seed}.yaml'
                    path.write_text(yaml.dump(data, Dumper=yaml.CSafeDumper, sort_keys=False, default_flow_style=None))
                    matrix = path.with_suffix('.matrix')
                    ids = [int(vertex_ids[tuple(points[s])]) for s in starts]
                    rows = [[int(vertex_ids[tuple(points[g])]) for g in row] for row in allowed]
                    matrix.write_text(f'# seed: {seed}\n# map: {map_path.resolve()}\n# n: {n}\nStart positions: ' + ' '.join(map(str, ids)) + '\n' + '\n'.join(' '.join(map(str, row)) for row in rows) + '\n')
                    cases.append({'case_id': f'{dirname}/{path.stem}', 'suite': suite,
                                  'map': stem, 'scenario': 'G' if shared is None else ratio,
                                  'num_agents': n, 'seed': seed,
                                  'fixture_file': str(path.resolve()), 'matrix_file': str(matrix.resolve()),
                                  'fixture_sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
            print(f'{dirname}: done, total={len(cases)}', flush=True)
    assert len(cases) == 9760
    manifest = {'paper': 'https://arxiv.org/abs/2609.14208v1',
                'original_instances_recovered': False,
                'original_sampling_equivalence_verified': False,
                'map_encoding': 'free cells normalized to . for all solver parsers',
                'generator': 'numpy RandomState(seed), largest connected component; unique private targets',
                'group_count': sum(c['suite'] == 'group' for c in cases),
                'common_count': sum(c['suite'] == 'common' for c in cases), 'cases': cases}
    (root / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    print('Verified 3200 group + 6560 common instances with reachable perfect-matching witnesses.', flush=True)

if __name__ == '__main__':
    raise SystemExit('Deprecated custom sampler. Use tools/prepare_repository_paper_2609.py, which invokes the existing exp1/exp2 scripts.')
