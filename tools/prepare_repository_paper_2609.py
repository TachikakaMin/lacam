"""Invoke existing repository generators, normalize map encoding, audit and index.

No sampling logic is implemented here. Original generator programs run unchanged.
"""
import argparse
import asyncio
import hashlib
import json
import os
import re
import sys
import time
from pathlib import Path

import numpy as np
import yaml
from scipy.ndimage import label
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import maximum_bipartite_matching
from normalize_symbotic_itacbs_fixtures import normalize_map_text
from run_ir_lacam_cross_experiment import yaml_to_matrix

REPO = Path(__file__).resolve().parents[1]
MAPS = {'random_32_32': 'random-32-32-10', 'symbotic_39_37': 'symbotic',
        'den312d_65_81': 'den312d', 'maze_32_32_2': 'maze-32-32-2',
        'room_64_64_8': 'room-64-64-8', 'warehouse_161_63': 'warehouse-10-20-10-2-1',
        'orz900d': 'orz900d', 'Boston': 'Boston_0_256'}

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

async def prepare(root, jobs, group_seed, reuse_generated=False, group_extension=False):
    if root.exists() and not reuse_generated:
        raise FileExistsError(f'Choose a new directory; refusing to overwrite {root}')
    if reuse_generated and not root.exists():
        raise FileNotFoundError(root)
    root.mkdir(parents=True, exist_ok=reuse_generated)
    (root / 'generator_logs').mkdir(exist_ok=reuse_generated)
    map_dir = root / 'normalized_maps'
    map_dir.mkdir(exist_ok=reuse_generated)
    allowed = sorted(os.sched_getaffinity(0))
    if len(allowed) <= 4:
        raise ValueError('Need more than four available CPUs')
    reserved = allowed[-4:]
    cpus = allowed[:-4]
    os.sched_setaffinity(0, cpus)
    programs = {suite: REPO / f'third_party/ITA-CBS2/python/generate_data_for_exp{number}.py'
                for suite, number in [('group', 1), ('common', 2)]}
    commands, tasks = [], []
    for name, stem in MAPS.items():
        source = REPO / ('tests/assets/symbotic.map' if stem == 'symbotic'
                         else f'third_party/ITA-CBS2/map_file/{stem}.map')
        text, _ = normalize_map_text(source.read_text(), source)
        normalized = map_dir / source.name
        if normalized.exists():
            assert normalized.read_text() == text, 'Existing map encoding changed'
        else:
            normalized.write_text(text)
        for suite, ratio in ([('group', None)] if group_extension else [('group', None), ('common', '000'), ('common', '030'), ('common', '060'), ('common', '100')]):
            dirname = f'Paper_{name}_gp_5' if suite == 'group' else f'paper_{name}_ratio_{ratio}'
            folder = root / dirname
            args = ['--map_path', str(normalized), '--output_dir', str(folder)]
            if group_extension:
                args += ['--agent_start', '250', '--agent_stop', '500', '--agent_step', '50']
            if suite == 'common':
                args += ['--common_ratio', {'000': '0', '030': '0.3', '060': '0.6', '100': '1'}[ratio],
                         '--agent_start', '5' if stem.startswith('maze') else '15',
                         '--agent_stop', '60', '--agent_step', '5']
                argv = [sys.executable, str(programs[suite]), *args]
            else:
                # Existing Group program is unseeded. Supply the initial RNG state
                # externally, without replacing or modifying its sampling code.
                launcher = ('import sys,runpy,numpy as np;np.random.seed(int(sys.argv[1]));'
                            'sys.argv=sys.argv[2:];runpy.run_path(sys.argv[0],run_name="__main__")')
                argv = [sys.executable, '-c', launcher, str(group_seed), str(programs[suite]), *args]
            tasks.append((suite, stem, ratio, folder, argv))
    sem = asyncio.Semaphore(min(jobs, len(cpus)))
    async def run(index, task):
        suite, stem, ratio, folder, argv = task
        async with sem:
            cpu = cpus[index % min(jobs, len(cpus))]
            command = ['taskset', '-c', str(cpu), *argv]
            commands.append({'folder': folder.name, 'argv': command})
            env = dict(os.environ, MPLBACKEND='Agg', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
            with (root / 'generator_logs' / (folder.name + '.log')).open('wb') as log:
                process = await asyncio.create_subprocess_exec(*command, stdout=log, stderr=log, env=env)
                if await process.wait() != 0:
                    raise RuntimeError(f'Generator failed: {folder.name}; inspect generator log')
            print(f'Generated {folder.name}: {len(list(folder.glob("*.yaml")))} fixtures', flush=True)
    if reuse_generated:
        for i, task in enumerate(tasks):
            suite, stem, ratio, folder, argv = task
            commands.append({'folder': folder.name,
                             'argv': ['taskset', '-c', str(cpus[i % min(jobs, len(cpus))]), *argv],
                             'record_source': 'reconstructed from initial invocation; see prepare.log'})
    else:
        await asyncio.gather(*(run(i, task) for i, task in enumerate(tasks)))
    cases, errors = [], []
    component_histogram, group_size_histogram = {}, {}
    for suite, stem, ratio, folder, argv in tasks:
        map_path = folder / (stem + '.map')
        rows = map_path.read_text().splitlines()[4:]
        assert set(''.join(rows)) <= {'.', '@', 'T'}
        grid = np.array([[c == '.' for c in row] for row in rows])
        labels, _ = label(grid)
        counts = (range(250, 501, 50) if group_extension else range(10, 201, 10)) if suite == 'group' else range(5 if stem.startswith('maze') else 15, 61, 5)
        expected = {(n, seed) for n in counts for seed in range(20)}
        actual = set()
        for fixture in sorted(folder.glob('*.yaml')):
            match = re.search(r'agents_(\d+)_test_(\d+)\.yaml$', fixture.name)
            n, seed = map(int, match.groups())
            actual.add((n, seed))
            data = yaml.load(fixture.read_text(), Loader=yaml.CSafeLoader)
            agents = data['agents']
            assert len(agents) == n and len({tuple(a['start']) for a in agents}) == n
            components = {int(labels[tuple(a['start'])]) for a in agents}
            component_histogram[str(len(components))] = component_histogram.get(str(len(components)), 0) + 1
            goal_ids, rr, cc = {}, [], []
            for i, a in enumerate(agents):
                start = tuple(a['start'])
                assert grid[start] and labels[start] > 0
                goals = [tuple(g) for g in a['potentialGoals']]
                assert len(goals) == len(set(goals))
                if suite == 'common': assert len(goals) == 16
                else: group_size_histogram[str(len(goals))] = group_size_histogram.get(str(len(goals)), 0) + 1
                for g in goals:
                    assert grid[g] and labels[g] == labels[start]
                    if g not in goal_ids: goal_ids[g] = len(goal_ids)
                    rr.append(i); cc.append(goal_ids[g])
            graph = csr_matrix((np.ones(len(rr)), (rr, cc)), shape=(n, len(goal_ids)))
            matching = maximum_bipartite_matching(graph, perm_type='column')
            if np.any(matching < 0): errors.append({'case_id': folder.name + '/' + fixture.stem, 'error': 'no perfect matching'})
            matrix = fixture.with_suffix('.matrix')
            yaml_to_matrix(fixture, matrix, None, seed)
            # Existing converter sorts each goal set. Verify exact set equality.
            from run_ir_lacam_cross_experiment import load_map
            info = load_map(map_path)
            lines = matrix.read_text().splitlines()
            assert list(map(int, lines[3].split(':', 1)[1].split())) == [info.coord_to_id[tuple(a['start'])] for a in agents]
            assert [set(map(int, line.split())) for line in lines[4:]] == [{info.coord_to_id[tuple(g)] for g in a['potentialGoals']} for a in agents]
            cases.append({'case_id': folder.name + '/' + fixture.stem, 'suite': suite, 'map': stem,
                          'scenario': ratio or 'G', 'num_agents': n, 'seed': seed,
                          'fixture_file': str(fixture), 'matrix_file': str(matrix),
                          'fixture_sha256': sha(fixture), 'matrix_sha256': sha(matrix), 'map_sha256': sha(map_path)})
        assert actual == expected
        print(f'Audited {folder.name}; cases={len(cases)}; matching errors={len(errors)}', flush=True)
    audit = {'complete': len(cases) == (960 if group_extension else 9760) and not errors, 'cases': len(cases),
             'perfect_matching_errors': errors, 'map_parser_semantics_equal': True,
             'yaml_matrix_goal_sets_equal': True, 'components_per_case_histogram': component_histogram,
             'group_candidate_count_histogram': group_size_histogram}
    (root / 'dataset_audit.json').write_text(json.dumps(audit, indent=2))
    manifest = {'paper': 'https://arxiv.org/abs/2609.14208v1', 'original_instances_recovered': False,
                'generator': 'Existing repository exp1/exp2 programs, unchanged sampling logic',
                'generation_policy': {'common': 'original np.random.seed(1), continuous stream',
                                      'group': f'original script with external initial seed {group_seed}; original historical RNG state unavailable',
                                      'solver_seed': 'test index 0..19, separate from generation RNG',
                                      'map_encoding': 'all free cells normalized to . before generation'},
                'generator_sha256': {k: sha(v) for k, v in programs.items()},
                'generator_commands': commands, 'reserved_cpus': reserved,
                'group_count': sum(c['suite'] == 'group' for c in cases),
                'common_count': sum(c['suite'] == 'common' for c in cases), 'cases': cases}
    (root / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    if not audit['complete']:
        raise RuntimeError('Original generated dataset failed audit; instances preserved, no silent resampling')
    print(f'READY: {len(cases)} audited instances; existing generator sampling unchanged', flush=True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--jobs', type=int, default=8)
    parser.add_argument('--group-initial-seed', type=int, default=1)
    parser.add_argument('--group-extension', action='store_true')
    parser.add_argument('--reuse-generated', action='store_true', help='Audit existing output without sampling again')
    args = parser.parse_args()
    asyncio.run(prepare(args.out_dir.resolve(), args.jobs, args.group_initial_seed, args.reuse_generated, args.group_extension))
