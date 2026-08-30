"""Golden transition corpus — Python side (debug.md round-2 P1-8).

Shared corpus: tests/fixtures/golden/*.{yaml,trans}.  Each .trans line is
`legal|illegal  tok;tok;...` applied SEQUENTIALLY: legal lines must be
accepted and advance the state; illegal lines must raise and leave the
state untouched.  The C++ validator runs the same corpus
(tests/test_dd_golden.cpp); together they pin both implementations to one
transition semantics (design.md 6.4 anti-drift).
"""

import unittest
from pathlib import Path

from ddbench.instance import load_instance
from ddbench.validator import TransitionError, apply_joint_action, initial_state

GOLDEN = Path(__file__).resolve().parent.parent.parent / "tests/fixtures/golden"


def parse_trans(path):
    lines = []
    for raw in Path(path).read_text().splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        verdict, rest = line.split(None, 1)
        joint = []
        for tok in rest.split(";"):
            parts = tok.split()
            if parts[0] == "w":
                joint.append(("wait",))
            elif parts[0] == "m":
                joint.append(("move", (int(parts[1]), int(parts[2]))))
            elif parts[0] == "l":
                joint.append(("lift",))
            elif parts[0] == "d":
                joint.append(("drop",))
            else:
                raise ValueError(f"bad token {tok!r} in {path}")
        lines.append((verdict, joint, raw))
    return lines


class TestGoldenCorpus(unittest.TestCase):
    def test_corpus_exists_and_is_nontrivial(self):
        cases = sorted(GOLDEN.glob("*.trans"))
        self.assertGreaterEqual(len(cases), 3)
        n_lines = sum(len(parse_trans(c)) for c in cases)
        self.assertGreaterEqual(n_lines, 20)

    def test_all_cases(self):
        for trans_path in sorted(GOLDEN.glob("*.trans")):
            yaml_path = trans_path.with_suffix(".yaml")
            with self.subTest(case=trans_path.stem):
                ins = load_instance(yaml_path)
                s = initial_state(ins)
                for k, (verdict, joint, raw) in enumerate(
                        parse_trans(trans_path)):
                    if verdict == "legal":
                        try:
                            s = apply_joint_action(ins, s, joint)
                        except TransitionError as e:
                            self.fail(
                                f"{trans_path.stem}:{k} expected legal, "
                                f"rejected: {e}\n  line: {raw}")
                    else:
                        with self.assertRaises(
                                TransitionError,
                                msg=f"{trans_path.stem}:{k} expected "
                                    f"illegal, accepted\n  line: {raw}"):
                            apply_joint_action(ins, s, joint)


if __name__ == "__main__":
    unittest.main()
