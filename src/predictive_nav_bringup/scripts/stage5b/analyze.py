#!/usr/bin/env python3
"""
Stage-5B classification metrics: confusion matrices, per-class P/R/F1, failures.

TWO SEPARATE QUESTIONS, KEPT SEPARATE
-------------------------------------
1. DETECTION COVERAGE. Did the frozen pipeline produce a track for this actor
   at all? An object that is never clustered is never classified, and folding
   that into a classification score would silently flatter the classifier --
   a class that is invisible would simply not appear in the matrix. Coverage
   is therefore reported first and independently, per actor and pattern, from
   the ground-truth record of which scenarios actually ran.

2. CLASSIFICATION. Given a track, what label did it get? Reported at two
   granularities, because they answer different things:

   PER-DETECTION  one sample per (scan, track) on scans where the track was
                  associated and real cluster geometry existed. Measures the
                  raw instantaneous decision, before any smoothing.
   PER-TRACK      one sample per (scenario, track_id), using the STABILISED
                  label the track held on the largest number of published
                  scans. This is the headline result: it is what a consumer
                  of /tracked_objects actually sees, and the stage's purpose
                  is stable per-object classification rather than per-scan
                  accuracy.

Ground truth is always the actor's model identity (`gt_class`), recorded by
the runner from actors.gt_class_of() and never derived from classifier output.

UNKNOWN IS SCORED AS A REAL CLASS, NOT AS AN ABSTENTION. The planter actor has
ground truth UNKNOWN, so UNKNOWN has a precision and recall like any other
label. The UNKNOWN RATE -- how often each true class gets declined -- is
reported alongside, because for the three target classes a high UNKNOWN rate is
the cost of the classifier's refusal to guess and must be visible.
"""
import argparse
from collections import Counter, defaultdict
import glob
import gzip
import json
import os

CLASSES = ('PEDESTRIAN', 'BIKE', 'VEHICLE', 'UNKNOWN')


def load(paths):
    """Read every JSONL row from `paths`, transparently handling .jsonl.gz."""
    rows = []
    for path in paths:
        opener = gzip.open if path.endswith('.gz') else open
        with opener(path, 'rt') as handle:
            for line in handle:
                line = line.strip()
                if line:
                    rows.append(json.loads(line))
    return rows


def confusion(pairs):
    """Confusion counts {(truth, predicted): n} from (truth, predicted) pairs."""
    matrix = Counter()
    for truth, pred in pairs:
        matrix[(truth, pred)] += 1
    return matrix


def render_matrix(matrix, title):
    """Format a confusion matrix with rows = truth, columns = prediction."""
    lines = [f'{title}', '  rows = GROUND TRUTH, columns = PREDICTED', '']
    head = '  ' + f'{"true \\\\ pred":>14s}' + ''.join(f'{c:>12s}' for c in CLASSES)
    lines += [head, '  ' + '-' * (14 + 12 * len(CLASSES))]
    for truth in CLASSES:
        total = sum(matrix[(truth, p)] for p in CLASSES)
        if total == 0:
            continue
        row = '  ' + f'{truth:>14s}'
        for pred in CLASSES:
            row += f'{matrix[(truth, pred)]:>12d}'
        lines.append(row + f'   (n={total})')
    return '\n'.join(lines)


def prf(matrix):
    """Per-class precision, recall, F1 and support from a confusion matrix."""
    out = {}
    for c in CLASSES:
        tp = matrix[(c, c)]
        fp = sum(matrix[(t, c)] for t in CLASSES if t != c)
        fn = sum(matrix[(c, p)] for p in CLASSES if p != c)
        support = tp + fn
        precision = tp / (tp + fp) if (tp + fp) else float('nan')
        recall = tp / support if support else float('nan')
        if precision == precision and recall == recall and (precision + recall) > 0:
            f1 = 2 * precision * recall / (precision + recall)
        else:
            f1 = float('nan')
        out[c] = (precision, recall, f1, support)
    return out


def render_prf(table, title):
    """Format a precision/recall/F1 table."""
    lines = [title, '',
             f'  {"":>12s}{"Precision":>12s}{"Recall":>10s}{"F1":>8s}{"Support":>10s}']
    lines.append('  ' + '-' * 52)
    for c in CLASSES:
        p, r, f, n = table[c]
        if n == 0 and p != p:
            continue
        fmt = (lambda v: '     n/a' if v != v else f'{v:8.3f}')
        lines.append(f'  {c:>12s}{fmt(p):>12s}{fmt(r):>10s}{fmt(f):>8s}{n:>10d}')
    return '\n'.join(lines)


def track_label(rows):
    """Stabilised label a track held on the most published scans; UNKNOWN on a tie."""
    counts = Counter(r['pred_class'] for r in rows)
    top = counts.most_common()
    if len(top) > 1 and top[0][1] == top[1][1]:
        return 'UNKNOWN'
    return top[0][0]


def main():
    """Compute and print the Stage-5B classification report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', required=True, help='directory of *.jsonl')
    parser.add_argument('--out', default='', help='optional path for the report')
    args = parser.parse_args()

    paths = sorted(glob.glob(os.path.join(args.raw, '*.jsonl')) +
                   glob.glob(os.path.join(args.raw, '*.jsonl.gz')))
    rows = load(paths)
    if not rows:
        raise SystemExit(f'no rows found under {args.raw}')

    report = []

    def emit(text=''):
        report.append(text)
        print(text)

    emit('=' * 78)
    emit('STAGE-5B CLASSIFICATION METRICS')
    emit('=' * 78)
    emit(f'raw files : {len(paths)}')
    emit(f'rows read : {len(rows)}')

    usable = [r for r in rows if r.get('scenario_ok', True)]
    emit(f'rows in scenarios that actually ran (ground-truth verified): {len(usable)}')

    # ---------------------------------------------------------------- coverage
    emit()
    emit('-' * 78)
    emit('1. DETECTION COVERAGE -- did the frozen pipeline track the actor at all?')
    emit('-' * 78)
    emit('  A scenario with 0 tracked scans produced NO classification sample. It is')
    emit('  a detection failure of the clustering envelope, not a classifier error,')
    emit('  and is excluded from the matrices below and reported only here.')
    emit()
    by_scen = defaultdict(list)
    for r in usable:
        by_scen[r['scenario']].append(r)
    actor_pat = defaultdict(lambda: [0, 0])
    for scen, srows in by_scen.items():
        actor, pattern, _ = scen.split('__')
        actor_pat[(actor, pattern)][1] += 1
        if srows:
            actor_pat[(actor, pattern)][0] += 1
    emit(f'  {"actor":>24s}{"pattern":>12s}{"scenarios":>12s}{"with a track":>14s}')
    emit('  ' + '-' * 62)
    for (actor, pattern) in sorted(actor_pat):
        got, tot = actor_pat[(actor, pattern)]
        emit(f'  {actor:>24s}{pattern:>12s}{tot:>12d}{got:>14d}')

    # Scenarios that ran but were never detected need the scenario list, which
    # only exists for scenarios that produced rows; recover the rest from the
    # scenario definitions.
    try:
        from scenarios import scenarios as all_scenarios
        defined = {s['id']: s for s in all_scenarios()}
        never = sorted(set(defined) - set(by_scen))
        if never:
            emit()
            emit(f'  NEVER DETECTED ({len(never)} of {len(defined)} scenarios) -- '
                 'no cluster survived, so no track and no label:')
            for sid in never:
                emit(f'    {sid}  (gt={defined[sid]["gt_class"]})')
    except Exception as exc:                                  # noqa: BLE001
        emit(f'  (scenario definitions unavailable: {exc})')

    # ---------------------------------------------------------- per-detection
    emit()
    emit('-' * 78)
    emit('2. PER-DETECTION CLASSIFICATION (unsmoothed, one sample per scan)')
    emit('-' * 78)
    det = [r for r in usable if r.get('features_valid')]
    emit(f'  samples: {len(det)} (scans on which the track was associated and real')
    emit('           cluster geometry existed; coasting scans carry no geometry)')
    emit()
    det_m = confusion([(r['gt_class'], r['pred_class_instant']) for r in det])
    emit(render_matrix(det_m, '  CONFUSION MATRIX -- per detection'))
    emit()
    emit(render_prf(prf(det_m), '  METRICS -- per detection'))
    emit()
    per_class_n = Counter(r['gt_class'] for r in det)
    emit('  samples per ground-truth class: ' +
         ', '.join(f'{c}={per_class_n.get(c, 0)}' for c in CLASSES))

    # -------------------------------------------------------------- per-track
    emit()
    emit('-' * 78)
    emit('3. PER-TRACK STABLE CLASSIFICATION  <-- HEADLINE RESULT')
    emit('-' * 78)
    tracks = defaultdict(list)
    for r in usable:
        tracks[(r['scenario'], r['track_id'])].append(r)
    pairs = []
    track_rows = []
    for (scen, tid), trows in sorted(tracks.items()):
        gt = trows[0]['gt_class']
        label = track_label(trows)
        pairs.append((gt, label))
        track_rows.append((scen, tid, gt, label, len(trows)))
    emit(f'  tracks: {len(pairs)}')
    emit()
    trk_m = confusion(pairs)
    emit(render_matrix(trk_m, '  CONFUSION MATRIX -- per stable track'))
    emit()
    emit(render_prf(prf(trk_m), '  METRICS -- per stable track'))
    emit()
    tn = Counter(gt for gt, _ in pairs)
    emit('  tracks per ground-truth class: ' +
         ', '.join(f'{c}={tn.get(c, 0)}' for c in CLASSES))

    # --------------------------------------------------------- UNKNOWN rates
    emit()
    emit('-' * 78)
    emit('4. UNKNOWN RATE -- how often the classifier declines to label')
    emit('-' * 78)
    for name, data, key in (('per detection', det, 'pred_class_instant'),
                            ('per stable track', None, None)):
        if data is not None:
            total = len(data)
            unk = sum(1 for r in data if r[key] == 'UNKNOWN')
            emit(f'  {name:18s} overall: {unk}/{total} = '
                 f'{(unk / total if total else float("nan")):.3f}')
            for c in CLASSES:
                sub = [r for r in data if r['gt_class'] == c]
                if sub:
                    u = sum(1 for r in sub if r[key] == 'UNKNOWN')
                    emit(f'  {"":18s}   true {c:<11s}: {u}/{len(sub)} = '
                         f'{u / len(sub):.3f}')
        else:
            total = len(pairs)
            unk = sum(1 for _, p in pairs if p == 'UNKNOWN')
            emit(f'  {name:18s} overall: {unk}/{total} = '
                 f'{(unk / total if total else float("nan")):.3f}')
            for c in CLASSES:
                sub = [p for gt, p in pairs if gt == c]
                if sub:
                    u = sum(1 for p in sub if p == 'UNKNOWN')
                    emit(f'  {"":18s}   true {c:<11s}: {u}/{len(sub)} = '
                         f'{u / len(sub):.3f}')

    # ------------------------------------------------------ failure analysis
    emit()
    emit('-' * 78)
    emit('5. FAILURE ANALYSIS -- the measured geometry behind each confusion')
    emit('-' * 78)
    emit('  For every (true, predicted) pair that actually occurred, the median of')
    emit('  the features the decision was made on. This is what explains a')
    emit('  confusion: the classifier is a conjunction of intervals on these.')
    emit()

    def med(vals):
        vals = sorted(vals)
        return vals[len(vals) // 2] if vals else float('nan')

    groups = defaultdict(list)
    for r in det:
        groups[(r['gt_class'], r['pred_class_instant'])].append(r)
    emit(f'  {"true -> predicted":>28s}{"n":>7s}{"len":>8s}{"thick":>8s}'
         f'{"fill":>8s}{"gap":>8s}{"pts":>7s}{"range":>8s}{"speed":>8s}')
    emit('  ' + '-' * 88)
    for key in sorted(groups, key=lambda k: -len(groups[k])):
        g = groups[key]
        emit(f'  {key[0] + " -> " + key[1]:>28s}{len(g):>7d}'
             f'{med([r["cluster_length"] for r in g]):>8.2f}'
             f'{med([r["cluster_thickness"] for r in g]):>8.2f}'
             f'{med([r["cluster_fill_ratio"] for r in g]):>8.2f}'
             f'{med([r["cluster_max_gap"] for r in g]):>8.2f}'
             f'{med([r["cluster_point_count"] for r in g]):>7.0f}'
             f'{med([r["cluster_range"] for r in g]):>8.2f}'
             f'{med([r["speed"] for r in g]):>8.2f}')

    emit()
    emit('  CLUSTER FRAGMENTATION -- how many distinct tracks one actor produced.')
    emit('  More than one track for a single physical object means the frozen')
    emit('  single-link clustering split it, and the classifier then sees FRAGMENTS')
    emit('  rather than the object. cluster_distance bounds this: a gap in an')
    emit('  object wider than cluster_distance becomes a cluster boundary.')
    emit()
    frag = defaultdict(set)
    for r in usable:
        frag[r['scenario']].add(r['track_id'])
    per_actor = defaultdict(list)
    for scen, tids in frag.items():
        per_actor[scen.split('__')[0]].append(len(tids))
    emit(f'  {"actor":>24s}{"scenarios":>12s}{"median tracks":>16s}{"max tracks":>13s}')
    emit('  ' + '-' * 65)
    for actor in sorted(per_actor):
        counts = sorted(per_actor[actor])
        emit(f'  {actor:>24s}{len(counts):>12d}'
             f'{counts[len(counts) // 2]:>16d}{max(counts):>13d}')

    emit()
    emit('  PER-ACTOR, PER-ASPECT geometry and labels. Pooling by class hides the')
    emit('  aspect dependence, which is the whole story for a 2D slice.')
    emit()
    emit(f'  {"actor":>24s}{"pattern":>10s}{"n":>6s}{"len":>7s}{"thick":>7s}'
         f'{"fill":>6s}{"gap":>6s}{"pts":>5s}{"rng":>6s}  instant labels')
    emit('  ' + '-' * 104)
    ap = defaultdict(list)
    for r in det:
        ap[(r['actor'], r['pattern'])].append(r)
    for key in sorted(ap):
        v = ap[key]
        labels = Counter(r['pred_class_instant'] for r in v).most_common(3)
        shown = ' '.join(f'{k}:{n}' for k, n in labels)
        emit(f'  {key[0]:>24s}{key[1]:>10s}{len(v):>6d}'
             f'{med([r["cluster_length"] for r in v]):>7.2f}'
             f'{med([r["cluster_thickness"] for r in v]):>7.2f}'
             f'{med([r["cluster_fill_ratio"] for r in v]):>6.2f}'
             f'{med([r["cluster_max_gap"] for r in v]):>6.2f}'
             f'{med([r["cluster_point_count"] for r in v]):>5.0f}'
             f'{med([r["cluster_range"] for r in v]):>6.2f}  {shown}')

    emit()
    emit('  UNKNOWN vs range, per true class (per-detection). Shows where the')
    emit('  sensor stops supporting a decision at all:')
    emit(f'  {"true class":>14s}{"range bin":>14s}{"n":>8s}{"UNKNOWN":>10s}{"rate":>8s}')
    emit('  ' + '-' * 54)
    bins = ((0, 2), (2, 3), (3, 4), (4, 5), (5, 99))
    for c in CLASSES:
        for lo, hi in bins:
            sub = [r for r in det
                   if r['gt_class'] == c and lo <= r['cluster_range'] < hi]
            if not sub:
                continue
            u = sum(1 for r in sub if r['pred_class_instant'] == 'UNKNOWN')
            label = f'{lo}-{hi} m' if hi < 99 else f'{lo}+ m'
            emit(f'  {c:>14s}{label:>14s}{len(sub):>8d}{u:>10d}'
                 f'{u / len(sub):>8.2f}')

    emit()
    emit('  fill_ratio distribution, to confirm the estimator stays within its')
    emit('  definitional bound of 1.0:')
    fills = sorted(r['cluster_fill_ratio'] for r in det)
    if fills:
        over = sum(1 for v in fills if v > 1.0)
        emit(f'    n={len(fills)}  min={fills[0]:.3f}  '
             f'median={fills[len(fills) // 2]:.3f}  max={fills[-1]:.3f}  '
             f'above 1.0: {over}')

    emit()
    emit('  Measured speed by true class (per-detection), which is why speed')
    emit('  carries almost no discriminative information in this dataset:')
    emit(f'  {"true class":>14s}{"n":>8s}{"min":>8s}{"median":>8s}{"max":>8s}')
    emit('  ' + '-' * 46)
    for c in CLASSES:
        sub = [r['speed'] for r in det if r['gt_class'] == c]
        if sub:
            emit(f'  {c:>14s}{len(sub):>8d}{min(sub):>8.2f}'
                 f'{med(sub):>8.2f}{max(sub):>8.2f}')

    # ------------------------------------------- Stage-5B.1 compound section
    emit()
    emit('-' * 78)
    emit('6. STAGE-5B.1 COMPOUND-OBSERVATION RECOVERY')
    emit('-' * 78)
    emit('  A COMPOUND OBSERVATION is the pooled geometry of two primary')
    emit('  clusters that the gap-aware stage judged to be two slices of ONE')
    emit('  rigid object. `from_compound` is read straight off the message, so')
    emit('  true and FALSE compound associations are both counted, never')
    emit('  inferred from the resulting geometry.')
    emit()
    comp = [r for r in det if r.get('from_compound')]
    if not any('from_compound' in r for r in det):
        emit('  NOT RECORDED in this dataset (pre-Stage-5B.1 raw rows).')
    else:
        emit(f'  compound detections : {len(comp)} of {len(det)}')
        emit()
        emit('  BY TRUE CLASS -- a compound association on anything but BIKE is')
        emit('  a FALSE compound association, which is the quantity the')
        emit('  anti-merge requirement bounds.')
        emit(f'  {"true class":>14s}{"detections":>12s}{"compound":>10s}'
             f'{"rate":>8s}{"verdict":>22s}')
        emit('  ' + '-' * 68)
        false_compound = 0
        for c in CLASSES:
            sub = [r for r in det if r['gt_class'] == c]
            cs = [r for r in sub if r.get('from_compound')]
            if not sub:
                continue
            if c == 'BIKE':
                verdict = 'recovered (intended)'
            else:
                verdict = 'FALSE COMPOUND' if cs else 'none (correct)'
                false_compound += len(cs)
            emit(f'  {c:>14s}{len(sub):>12d}{len(cs):>10d}'
                 f'{len(cs) / len(sub):>8.3f}{verdict:>22s}')
        emit()
        emit(f'  FALSE compound detections (true class != BIKE): {false_compound}')
        emit()

        # Per-scenario recovery accounting for the bike actor.
        emit('  BIKE RECOVERY PER SCENARIO. `frag` counts scans on which the')
        emit('  actor was still reported as two separate fragment tracks, i.e.')
        emit('  compound association had not yet reached its support threshold')
        emit('  or did not fire.')
        emit(f'  {"scenario":>34s}{"scans":>7s}{"compound":>10s}{"frag":>7s}'
             f'{"tracks":>8s}{"stable labels":>28s}')
        emit('  ' + '-' * 94)
        bike_scen = sorted({r['scenario'] for r in det if r['gt_class'] == 'BIKE'})
        for scen in bike_scen:
            sub = [r for r in det if r['scenario'] == scen]
            cs = [r for r in sub if r.get('from_compound')]
            ids = {r['track_id'] for r in sub}
            labels = Counter(r['pred_class'] for r in sub)
            lab = ' '.join(f'{k}:{v}' for k, v in labels.most_common())
            emit(f'  {scen:>34s}{len(sub):>7d}{len(cs):>10d}'
                 f'{len(sub) - len(cs):>7d}{len(ids):>8d}{lab:>28s}')
        emit()

        # Geometry of the recovered compound observations.
        if comp:
            emit('  GEOMETRY OF THE RECOVERED COMPOUND OBSERVATIONS. This is the')
            emit('  evidence that the BIKE label comes from a bicycle-shaped')
            emit('  measurement and not from relabelling a pedestrian track.')
            emit(f'  {"feature":>22s}{"min":>9s}{"median":>9s}{"max":>9s}')
            emit('  ' + '-' * 49)
            for key, name in (('cluster_length', 'length m'),
                              ('cluster_thickness', 'thickness m'),
                              ('cluster_fill_ratio', 'fill_ratio'),
                              ('cluster_max_gap', 'max_gap m'),
                              ('cluster_point_count', 'point_count'),
                              ('cluster_range', 'range m')):
                v = [r[key] for r in comp]
                emit(f'  {name:>22s}{min(v):>9.3f}{med(v):>9.3f}{max(v):>9.3f}')
            emit()
            emit('  For reference, the SAME actor before compound association')
            emit('  (one wheel fragment): length 0.51 m, thickness 0.03 m,')
            emit('  fill_ratio 1.00, max_gap 0.07 m -- pedestrian-sized and')
            emit('  solid, which is exactly why it classified PEDESTRIAN.')
            emit()
            lab = Counter(r['pred_class'] for r in comp)
            emit(f'  STABLE LABEL ON COMPOUND DETECTIONS: '
                 f'{dict(lab.most_common())}')

    # --------------------------------------------------------- per-track list
    emit()
    emit('-' * 78)
    emit('7. PER-TRACK DETAIL')
    emit('-' * 78)
    emit(f'  {"scenario":>46s}{"id":>5s}{"truth":>12s}{"stable":>12s}{"scans":>7s}')
    emit('  ' + '-' * 82)
    for scen, tid, gt, label, n in track_rows:
        flag = '' if gt == label else '   <-- MISMATCH'
        emit(f'  {scen:>46s}{tid:>5d}{gt:>12s}{label:>12s}{n:>7d}{flag}')

    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, 'w') as handle:
            handle.write('\n'.join(report) + '\n')
        print(f'\nwrote {args.out}')


if __name__ == '__main__':
    main()
