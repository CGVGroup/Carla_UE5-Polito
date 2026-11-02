#!/usr/bin/env python3
"""
aggregate_study.py: the tables and figures of protocol XAI-HUD-EXP-01 v2.0 (Section 12 analysis plan,
Appendix C) over all the drives of the study, from the drives analysed by tools/analyze_session.py (their
paper/metrics.json, with a participant code) and the questionnaires typed in after the sessions.

Inputs (study folder, default leaderboard/study/; empty templates: --init):
    the drives                logs/sessions/*/paper/metrics.json analysed with --participant (and --drive)
    questionnaires.csv        long format: participant, drive, instrument, item, value
                                SSQ     drive = pre / D1 / D2 / D3 / release, items 1..16 (Kennedy et al., 1993 order),
                                        values 0..3
                                TLX     drive = D1..D3, items mental, physical, temporal, performance, effort,
                                        frustration, values 0..100 (raw TLX)
                                SUS     items 1..10, values 1..5
                                TiA     items = subscale names (reliability_competence, understanding_predictability,
                                        familiarity, intention_of_developers, propensity_to_trust,
                                        trust_in_automation), values = subscale scores 1..5 scored with Körber's
                                        (2019) key (reverse-coded items included)
                                ExpSat  items 1..8, values 1..5 (A and T drives only)
    ranking.csv               participant, N, A, T: forced-choice rank of each interface (1 = best)
    trust_ratings.csv         optional dynamic-trust module: participant, drive, time_s (route time [s], as in
                              the session's report), rating 0..10

Outputs (study/results/):
    study_report.md                 all the tables below, readable
    table_design.csv                drives found: participant, drive, interface, route vs. the counterbalancing plan
    table_phase_e_profile.csv       RQ-1, participant x interface: monitoring ratio, display glances > 2 s, TiA,
                                    TLX, SUS, ExpSat (+ per interface median [range] in the report)
    table_omission_events.csv       RQ-2, every omission: detection, reaction time, gaze latency, min TTC,
                                    collision, peak decel., trust drop
    table_omission_response.csv     RQ-2, per interface: detection rate, RT, gaze latency, min TTC, collisions
    table_calibration.csv           RQ-3, participant x interface: observed reliability, unnecessary interventions,
                                    TiA trust, dynamic-trust drop
    table_attentional_tunnelling.csv  RQ-4, gaze latency per interface x omission 1 / 2
    table_ssq.csv                   Appendix C, SSQ trajectory (N, O, D, total, delta pre, stop rule > 20)
    table_ranking.csv               win rate per interface pair, Friedman chi2, Kendall's W (descriptive)
    fig_*.png                       per-event plots of RT, gaze latency, min TTC; detection rate; monitoring
                                    ratio; questionnaires; SSQ

Descriptive only (n = 5, protocol Section 12 note): medians and ranges, no significance tests.

Usage:
    python tools/aggregate_study.py --init        (empty questionnaire / ranking templates in study/)
    python tools/aggregate_study.py [--study <folder>] [--sessions <folder>]
"""
import argparse
import csv
import glob
import json
import os
import statistics
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
LEADERBOARD = os.path.dirname(TOOLS)
INTERFACES = ['N', 'A', 'T']
COLORS = {'N': '#7f7f7f', 'A': '#1f77b4', 'T': '#ff7f0e'}
# Section 5: interface · route of each drive
PLAN = {
    'P1': [('N', 'R1'), ('A', 'R2'), ('T', 'R3')],
    'P2': [('A', 'R3'), ('T', 'R1'), ('N', 'R2')],
    'P3': [('T', 'R2'), ('N', 'R3'), ('A', 'R1')],
    'P4': [('N', 'R3'), ('T', 'R2'), ('A', 'R1')],
    'P5': [('A', 'R2'), ('N', 'R1'), ('T', 'R3')],
    'P6': [('T', None), ('A', None), ('N', None)],  # missing Williams sequence T-A-N (routes to be set)
}
# SSQ items (Kennedy et al., 1993): 1 general discomfort, 2 fatigue, 3 headache, 4 eyestrain, 5 difficulty
# focusing, 6 increased salivation, 7 sweating, 8 nausea, 9 difficulty concentrating, 10 fullness of head,
# 11 blurred vision, 12 dizzy (eyes open), 13 dizzy (eyes closed), 14 vertigo, 15 stomach awareness, 16 burping
SSQ_N = [1, 6, 7, 8, 9, 15, 16]
SSQ_O = [1, 2, 3, 4, 5, 9, 11]
SSQ_D = [5, 8, 10, 11, 12, 13, 14]
SSQ_POINTS = ['pre', 'D1', 'D2', 'D3', 'release']
TLX_ITEMS = ['mental', 'physical', 'temporal', 'performance', 'effort', 'frustration']
TIA_SUBSCALES = ['reliability_competence', 'understanding_predictability', 'familiarity', 'intention_of_developers',
                 'propensity_to_trust', 'trust_in_automation']


def fnum(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def r(value, digits=3):
    return None if value is None else round(value, digits)


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline='', encoding='utf-8-sig') as f:
        return list(csv.DictReader(f))


def write_csv(path, rows, columns=None):
    columns = columns or (list(dict.fromkeys(k for row in rows for k in row)) if rows else ['none'])
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def med_range(values, digits=2):
    values = [v for v in values if v is not None]
    if not values:
        return '-'
    m = statistics.median(values)
    if len(values) == 1:
        return '{:.{d}f} (n=1)'.format(m, d=digits)
    return '{:.{d}f} [{:.{d}f}–{:.{d}f}] (n={})'.format(m, min(values), max(values), len(values), d=digits)


def fmt(value, digits=2):
    if value is None or value == '':
        return '-'
    if isinstance(value, bool):
        return 'yes' if value else 'no'
    if isinstance(value, float):
        return '{:.{}f}'.format(value, digits)
    return str(value)


def md_table(columns, rows):
    lines = ['| ' + ' | '.join(c for c, _ in columns) + ' |', '|' + '---|' * len(columns)]
    for row in rows:
        lines.append('| ' + ' | '.join(fmt(row.get(k)) for _, k in columns) + ' |')
    return '\n'.join(lines)


# ---------------------------------------------------------------------------------------------- inputs

def init_templates(study):
    os.makedirs(study, exist_ok=True)
    templates = {
        'questionnaires.csv': (['participant', 'drive', 'instrument', 'item', 'value'], [
            ['P1', 'pre', 'SSQ', i, ''] for i in range(1, 17)] + [
            ['P1', 'D1', 'TLX', item, ''] for item in TLX_ITEMS] + [
            ['P1', 'D1', 'SUS', i, ''] for i in range(1, 11)] + [
            ['P1', 'D1', 'TiA', s, ''] for s in TIA_SUBSCALES] + [
            ['P1', 'D2', 'ExpSat', i, ''] for i in range(1, 9)]),
        'ranking.csv': (['participant', 'N', 'A', 'T'], [['P1', '', '', '']]),
        'trust_ratings.csv': (['participant', 'drive', 'time_s', 'rating'], [['P1', '1', '', '']]),
    }
    for name, (header, rows) in templates.items():
        path = os.path.join(study, name)
        if os.path.exists(path):
            print('kept (already there):', path)
            continue
        with open(path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(rows)
        print('template:', path)


def load_drives(sessions_root, include_unnamed):
    drives = []
    for path in sorted(glob.glob(os.path.join(sessions_root, '*', 'paper', 'metrics.json'))):
        with open(path, encoding='utf-8') as f:
            m = json.load(f)
        if not m.get('protocol') or (not m.get('participant') and not include_unnamed):
            continue
        m['participant'] = m.get('participant') or '?'
        m['_path'] = path
        drives.append(m)
    # Drive numbers missing: by start time per participant
    by_participant = {}
    for m in drives:
        by_participant.setdefault(m['participant'], []).append(m)
    for items in by_participant.values():
        items.sort(key=lambda m: m.get('start') or '')
        for k, m in enumerate(items, 1):
            m['drive'] = str(m.get('drive') or k)
    return drives


def score_questionnaires(rows):
    """{(participant, drive): {'SSQ': {...}, 'TLX': x, 'SUS': x, 'TiA': {...}, 'ExpSat': x}}"""
    items = {}
    for row in rows:
        value = fnum(row.get('value'))
        if value is None:
            continue
        drive = str(row.get('drive', '')).strip()
        drive = drive[1:] if drive.upper().startswith('D') and drive[1:].isdigit() else drive
        key = (row.get('participant', '').strip(), drive)
        items.setdefault(key, {}).setdefault(row.get('instrument', '').strip(), {})[str(row.get('item', '')).strip()] = value
    scores = {}
    for key, instruments in items.items():
        s = {}
        ssq = instruments.get('SSQ')
        if ssq:
            raw = lambda ids: sum(ssq.get(str(i), 0.0) for i in ids)  # noqa: E731
            n, o, d = raw(SSQ_N), raw(SSQ_O), raw(SSQ_D)
            s['SSQ'] = {'N': r(n * 9.54, 1), 'O': r(o * 7.58, 1), 'D': r(d * 13.92, 1), 'total': r((n + o + d) * 3.74, 1),
                        'complete': len(ssq) >= 16}
        tlx = instruments.get('TLX')
        if tlx:
            values = [tlx[i] for i in TLX_ITEMS if i in tlx]
            s['TLX'] = r(sum(values) / len(values), 1) if len(values) == 6 else None
        sus = instruments.get('SUS')
        if sus:
            if all(str(i) in sus for i in range(1, 11)):  # a missing item invalidates the score
                s['SUS'] = r(2.5 * sum(sus[str(i)] - 1 if i % 2 else 5 - sus[str(i)] for i in range(1, 11)), 1)
            else:
                s['SUS'] = None
        tia = instruments.get('TiA')
        if tia:
            s['TiA'] = {k: tia.get(k) for k in TIA_SUBSCALES}
        exp = instruments.get('ExpSat')
        if exp:
            s['ExpSat'] = r(sum(exp.values()) / len(exp), 2)
            s['ExpSat_items'] = {k: v for k, v in sorted(exp.items())}
        scores[key] = s
    return scores


# ---------------------------------------------------------------------------------------------- tables

def omission_rows(drives, trust):
    rows = []
    for m in drives:
        ratings = sorted(((fnum(t['time_s']), fnum(t['rating'])) for t in trust
                          if t.get('participant') == m['participant'] and str(t.get('drive')) == m['drive']
                          and fnum(t.get('time_s')) is not None and fnum(t.get('rating')) is not None))
        for e in m['protocol']['events']:
            if not e.get('omission'):
                continue
            before = [v for t, v in ratings if t < e['t0']]
            after = [v for t, v in ratings if t >= e['t0']]
            rows.append({
                'participant': m['participant'], 'drive': m['drive'], 'interface': m['condition'],
                'route': m.get('route_code'), 'omission': e.get('omission_index'), 'event': e['event'],
                'scenario': e['scenario'], 'actors': e['actors'], 'detected': e['driver_reaction'],
                'reaction_time_s': e['reaction_time'], 'reaction_input': e['reaction_input'],
                'gaze_latency_s': e['gaze_latency'], 'min_ttc_s': e['min_ttc'], 'collision': e['collision'],
                'peak_decel': e['peak_decel_after_reaction'], 'peak_lat_acc': e['peak_lat_acc_after_reaction'],
                'lane_exceedance': e['lane_exceedance'],
                'trust_drop': r(after[0] - before[-1], 1) if before and after else None,
            })
    return rows


def friedman(ranks):
    """Friedman chi2 and Kendall's W of complete rankings [{interface: rank}]"""
    n, k = len(ranks), len(INTERFACES)
    if n < 2:
        return None, None
    sums = [sum(rk[i] for rk in ranks) for i in INTERFACES]
    chi2 = 12.0 / (n * k * (k + 1)) * sum(s * s for s in sums) - 3.0 * n * (k + 1)
    return r(chi2, 2), r(chi2 / (n * (k - 1)), 3)


# ---------------------------------------------------------------------------------------------- figures

def strip_figure(plt, rows, key, label, path, by_omission=True):
    fig, ax = plt.subplots(figsize=(5.5, 4))
    for x, interface in enumerate(INTERFACES):
        part = [row for row in rows if row['interface'] == interface and row.get(key) is not None]
        for k, row in enumerate(part):
            marker = {1: 'o', 2: '^'}.get(row.get('omission'), 's') if by_omission else 'o'
            ax.scatter(x + (k - (len(part) - 1) / 2) * 0.06, row[key], color=COLORS[interface], marker=marker,
                       edgecolor='k', lw=0.4, zorder=3)
            ax.annotate(row['participant'], (x + (k - (len(part) - 1) / 2) * 0.06, row[key]), fontsize=6,
                        xytext=(4, 0), textcoords='offset points')
        if part:
            m = statistics.median(row[key] for row in part)
            ax.hlines(m, x - 0.25, x + 0.25, color='k', lw=1.5)
        missing = sum(1 for row in rows if row['interface'] == interface and row.get(key) is None)
        if missing:
            ax.text(x, ax.get_ylim()[0], '{} none'.format(missing), ha='center', va='bottom', fontsize=7)
    ax.set_xticks(range(len(INTERFACES)))
    ax.set_xticklabels(['N (no AR HUD)', 'A (AR HUD)', 'T (text)'])
    ax.set_ylabel(label)
    if by_omission:
        ax.set_title('per omission event (o: omission 1, ^: omission 2; bar: median)', fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--study', default=os.path.join(LEADERBOARD, 'study'), help='questionnaires / ranking folder')
    ap.add_argument('--sessions', default=os.environ.get('SESSION_LOG_DIR') or os.path.join(LEADERBOARD, 'logs', 'sessions'))
    ap.add_argument('--init', action='store_true', help='write empty templates in the study folder')
    ap.add_argument('--include-unnamed', action='store_true', help='also the drives analysed without --participant')
    args = ap.parse_args()

    if args.init:
        init_templates(args.study)
        return 0
    drives = load_drives(args.sessions, args.include_unnamed)
    if not drives:
        print('No analysed drive with a participant code in', args.sessions,
              '(run tools/analyze_session.py <session> --participant P1 --drive 1)')
        return 2
    out = os.path.join(args.study, 'results')
    os.makedirs(out, exist_ok=True)
    scores = score_questionnaires(read_csv(os.path.join(args.study, 'questionnaires.csv')))
    trust = read_csv(os.path.join(args.study, 'trust_ratings.csv'))
    ranking_rows = read_csv(os.path.join(args.study, 'ranking.csv'))
    report = ['# XAI-HUD-EXP-01 — study results', '',
              '{} drives, participants {}. Descriptive only (protocol Section 12: medians and ranges, no '
              'significance tests).'.format(len(drives), ', '.join(sorted({m['participant'] for m in drives}))), '']

    # Design check (Section 5)
    design = []
    for m in sorted(drives, key=lambda m: (m['participant'], m['drive'])):
        plan = PLAN.get(m['participant'])
        planned = plan[int(m['drive']) - 1] if plan and m['drive'].isdigit() and 0 < int(m['drive']) <= len(plan) else None
        ok = planned is not None and planned[0] == m['condition'] and (planned[1] is None or planned[1] == m.get('route_code'))
        s = m['protocol']['summary']
        design.append({'participant': m['participant'], 'drive': m['drive'], 'interface': m['condition'],
                       'route': m.get('route_code'), 'planned': '{} · {}'.format(*planned) if planned else '-',
                       'as_planned': ok, 'events': s['events'], 'omissions': s['omissions'],
                       'session': m['session']})
    write_csv(os.path.join(out, 'table_design.csv'), design)
    report += ['## Drives and counterbalancing (Section 5)', '',
               md_table([('Participant', 'participant'), ('Drive', 'drive'), ('Interface', 'interface'), ('Route', 'route'),
                         ('Planned', 'planned'), ('As planned', 'as_planned'), ('Events', 'events'),
                         ('Omissions', 'omissions'), ('Session', 'session')], design), '',
               'The protocol expects 8 events with 2 omissions per drive.', '']

    # RQ-1: Phase E profile
    profile = []
    for m in drives:
        g = ((m['protocol'].get('gaze') or {}).get('E')) or {}
        q = scores.get((m['participant'], m['drive']), {})
        tia = q.get('TiA') or {}
        profile.append({'participant': m['participant'], 'drive': m['drive'], 'interface': m['condition'],
                        'monitoring_ratio': g.get('monitoring_ratio'), 'display_ratio': g.get('display_ratio'),
                        'display_glances': g.get('display_glances'),
                        'display_glances_over_2s_ratio': g.get('display_glances_over_2s_ratio'),
                        'phase_e_gaze_s': g.get('gaze_time_s'),
                        'tia_trust': tia.get('trust_in_automation'), 'tia_reliability': tia.get('reliability_competence'),
                        'tia_understanding': tia.get('understanding_predictability'),
                        'tlx': q.get('TLX'), 'sus': q.get('SUS'),
                        'expsat': q.get('ExpSat') if m['condition'] in ('A', 'T') else None})
    write_csv(os.path.join(out, 'table_phase_e_profile.csv'), profile)
    measures = [('Monitoring ratio (Phase E)', 'monitoring_ratio'), ('Display glances > 2 s (share)', 'display_glances_over_2s_ratio'),
                ('TiA trust', 'tia_trust'), ('TiA reliability/competence', 'tia_reliability'),
                ('TiA understanding/predictability', 'tia_understanding'), ('NASA-TLX raw', 'tlx'), ('SUS', 'sus'),
                ('Explanation Satisfaction', 'expsat')]
    summary_rows = [{'measure': label, **{i: med_range([p[key] for p in profile if p['interface'] == i]) for i in INTERFACES}}
                    for label, key in measures]
    report += ['## RQ-1 — Phase E profile (median [range] per interface)', '',
               md_table([('Measure', 'measure')] + [(i, i) for i in INTERFACES], summary_rows), '',
               'Per participant x interface: table_phase_e_profile.csv. Monitoring ratio: share of the Phase E gaze time '
               'on the road scene (display: text panel for T, AR-highlighted road users for A; in-vehicle: below '
               'the view). Questionnaires from questionnaires.csv.', '']

    # RQ-2: omission response
    omissions = omission_rows(drives, trust)
    write_csv(os.path.join(out, 'table_omission_events.csv'), omissions)
    response = []
    for i in INTERFACES:
        part = [o for o in omissions if o['interface'] == i]
        response.append({
            'interface': i, 'omission_events': len(part),
            'detected': '{}/{}'.format(sum(o['detected'] for o in part), len(part)) if part else '-',
            'detection_rate': r(sum(o['detected'] for o in part) / len(part)) if part else None,
            'reaction_time_s': med_range([o['reaction_time_s'] for o in part]),
            'gaze_latency_s': med_range([o['gaze_latency_s'] for o in part]),
            'min_ttc_s': med_range([o['min_ttc_s'] for o in part]),
            'collisions': sum(o['collision'] for o in part),
            'peak_decel': med_range([o['peak_decel'] for o in part]),
        })
    write_csv(os.path.join(out, 'table_omission_response.csv'), response)
    report += ['## RQ-2 — Omission response (10 omission events per interface expected: 5 x 2)', '',
               md_table([('Interface', 'interface'), ('Omissions', 'omission_events'), ('Detected', 'detected'),
                         ('Detection rate', 'detection_rate'), ('RT [s]', 'reaction_time_s'),
                         ('Gaze latency [s]', 'gaze_latency_s'), ('Min TTC [s]', 'min_ttc_s'),
                         ('Collisions', 'collisions'), ('Peak decel [m/s2]', 'peak_decel')], response), '',
               md_table([('P', 'participant'), ('Drive', 'drive'), ('IF', 'interface'), ('Route', 'route'),
                         ('Om.', 'omission'), ('Scenario', 'scenario'), ('Detected', 'detected'),
                         ('RT [s]', 'reaction_time_s'), ('Input', 'reaction_input'), ('Gaze lat. [s]', 'gaze_latency_s'),
                         ('Min TTC [s]', 'min_ttc_s'), ('Collision', 'collision'), ('Trust drop', 'trust_drop')], omissions),
               '', 'Per-event plots: fig_omission_reaction_time.png, fig_omission_gaze_latency.png, fig_omission_min_ttc.png.', '']

    # RQ-3: calibration
    calibration = []
    for m in drives:
        s = m['protocol']['summary']
        q = scores.get((m['participant'], m['drive']), {})
        drops = [o['trust_drop'] for o in omissions if o['participant'] == m['participant'] and o['drive'] == m['drive']
                 and o['trust_drop'] is not None]
        handled = s['events'] - s['omissions']
        calibration.append({'participant': m['participant'], 'drive': m['drive'], 'interface': m['condition'],
                            'observed_reliability': s['observed_reliability'],
                            'unnecessary_interventions': '{}/{}'.format(s['unnecessary_interventions'], handled),
                            'unnecessary_intervention_rate': s['unnecessary_intervention_rate'],
                            'tia_trust': (q.get('TiA') or {}).get('trust_in_automation'),
                            'tia_reliability': (q.get('TiA') or {}).get('reliability_competence'),
                            'mean_trust_drop': r(sum(drops) / len(drops), 2) if drops else None})
    write_csv(os.path.join(out, 'table_calibration.csv'), calibration)
    report += ['## RQ-3 — Calibration', '',
               md_table([('P', 'participant'), ('Drive', 'drive'), ('IF', 'interface'),
                         ('Observed reliability', 'observed_reliability'), ('Unnecessary interv.', 'unnecessary_interventions'),
                         ('TiA trust', 'tia_trust'), ('TiA reliability', 'tia_reliability'),
                         ('Trust drop (dynamic)', 'mean_trust_drop')],
                        sorted(calibration, key=lambda c: (c['participant'], c['drive']))), '',
               'Per interface, unnecessary-intervention rate: ' + '; '.join(
                   '{} {}'.format(i, med_range([c['unnecessary_intervention_rate'] for c in calibration if c['interface'] == i]))
                   for i in INTERFACES), '']

    # RQ-4: attentional tunnelling
    tunnelling = []
    for i in INTERFACES:
        row = {'interface': i}
        for k in (1, 2):
            row['omission_{}'.format(k)] = med_range([o['gaze_latency_s'] for o in omissions
                                                       if o['interface'] == i and o['omission'] == k])
        row['all'] = med_range([o['gaze_latency_s'] for o in omissions if o['interface'] == i])
        tunnelling.append(row)
    write_csv(os.path.join(out, 'table_attentional_tunnelling.csv'), tunnelling)
    report += ['## RQ-4 — Attentional tunnelling: gaze-to-hazard latency [s]', '',
               md_table([('Interface', 'interface'), ('Omission 1', 'omission_1'), ('Omission 2', 'omission_2'),
                         ('All', 'all')], tunnelling), '',
               'H2a (complacency): A/T slower than N; H2b (attentional support): A/T faster.', '']

    # Appendix C: SSQ trajectory
    ssq_rows = []
    for participant in sorted({p for p, _ in scores}):
        base = (scores.get((participant, 'pre'), {}).get('SSQ') or {}).get('total')
        for point in SSQ_POINTS:
            key = point[1:] if point.startswith('D') else point
            ssq = scores.get((participant, key), {}).get('SSQ')
            if not ssq:
                continue
            delta = r(ssq['total'] - base, 1) if base is not None and point != 'pre' else None
            ssq_rows.append({'participant': participant, 'point': point, 'N': ssq['N'], 'O': ssq['O'], 'D': ssq['D'],
                             'total': ssq['total'], 'delta_pre': delta, 'stop': delta is not None and delta > 20,
                             'complete': ssq['complete']})
    if ssq_rows:
        write_csv(os.path.join(out, 'table_ssq.csv'), ssq_rows)
        report += ['## Appendix C — SSQ trajectory', '',
                   md_table([('P', 'participant'), ('Point', 'point'), ('N', 'N'), ('O', 'O'), ('D', 'D'),
                             ('Total', 'total'), ('Δ pre', 'delta_pre'), ('Stop (Δ > 20)', 'stop')], ssq_rows), '']

    # Ranking
    ranks = []
    for row in ranking_rows:
        values = {i: fnum(row.get(i)) for i in INTERFACES}
        if all(v is not None for v in values.values()):
            ranks.append(values)
    if ranks:
        pairs = []
        for a in INTERFACES:
            for b in INTERFACES:
                if a < b:
                    wins = sum(rk[a] < rk[b] for rk in ranks)
                    pairs.append({'pair': '{} vs {}'.format(a, b), 'wins_first': wins, 'wins_second': len(ranks) - wins,
                                  'win_rate_first': r(wins / len(ranks))})
        chi2, w = friedman(ranks)
        rows = pairs + [{'pair': 'Friedman chi2 / Kendall W', 'wins_first': chi2, 'wins_second': w}]
        write_csv(os.path.join(out, 'table_ranking.csv'), rows)
        report += ['## Forced-choice ranking', '',
                   md_table([('Pair', 'pair'), ('Wins 1st', 'wins_first'), ('Wins 2nd', 'wins_second'),
                             ('Win rate 1st', 'win_rate_first')], pairs), '',
                   '- Mean rank: ' + ', '.join('{} {:.2f}'.format(i, sum(rk[i] for rk in ranks) / len(ranks)) for i in INTERFACES),
                   '- Friedman chi2 = {}, Kendall W = {} (n = {}, descriptive)'.format(chi2, w, len(ranks)), '']

    # Figures
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        figures = []
        for key, label, name in (('reaction_time_s', 'reaction time from t0 [s]', 'fig_omission_reaction_time.png'),
                                 ('gaze_latency_s', 'gaze-to-hazard latency [s]', 'fig_omission_gaze_latency.png'),
                                 ('min_ttc_s', 'min TTC [s]', 'fig_omission_min_ttc.png')):
            strip_figure(plt, omissions, key, label, os.path.join(out, name))
            figures.append(name)
        fig, ax = plt.subplots(figsize=(5, 3.5))
        ax.bar(INTERFACES, [row['detection_rate'] or 0 for row in response], color=[COLORS[i] for i in INTERFACES])
        for x, row in enumerate(response):
            ax.text(x, (row['detection_rate'] or 0) + 0.02, row['detected'], ha='center', fontsize=8)
        ax.set_ylim(0, 1.15)
        ax.set_ylabel('omissions detected by the driver')
        fig.tight_layout()
        fig.savefig(os.path.join(out, 'fig_omission_detection_rate.png'), dpi=150)
        plt.close(fig)
        figures.append('fig_omission_detection_rate.png')
        # Phase E profile: participants across interfaces
        panels = [(label, key) for label, key in measures if any(p[key] is not None for p in profile)]
        if panels:
            fig, axes = plt.subplots(1, len(panels), figsize=(3 * len(panels), 3.5), squeeze=False)
            for ax, (label, key) in zip(axes[0], panels):
                for participant in sorted({p['participant'] for p in profile}):
                    points = {p['interface']: p[key] for p in profile if p['participant'] == participant and p[key] is not None}
                    xs = [x for x, i in enumerate(INTERFACES) if i in points]
                    ax.plot(xs, [points[INTERFACES[x]] for x in xs], '-o', lw=0.8, ms=4, color='#999999')
                    if xs:
                        ax.annotate(participant, (xs[-1], points[INTERFACES[xs[-1]]]), fontsize=6, xytext=(3, 0),
                                    textcoords='offset points')
                for x, i in enumerate(INTERFACES):
                    values = [p[key] for p in profile if p['interface'] == i and p[key] is not None]
                    if values:
                        ax.hlines(statistics.median(values), x - 0.25, x + 0.25, color=COLORS[i], lw=2.5)
                ax.set_xticks(range(len(INTERFACES)))
                ax.set_xticklabels(INTERFACES)
                ax.set_title(label, fontsize=8)
            fig.tight_layout()
            fig.savefig(os.path.join(out, 'fig_phase_e_profile.png'), dpi=150)
            plt.close(fig)
            figures.append('fig_phase_e_profile.png')
        if ssq_rows:
            fig, ax = plt.subplots(figsize=(5.5, 3.5))
            for participant in sorted({row['participant'] for row in ssq_rows}):
                part = [row for row in ssq_rows if row['participant'] == participant]
                ax.plot([SSQ_POINTS.index(row['point']) for row in part], [row['total'] for row in part], '-o', label=participant)
            base = [row['total'] for row in ssq_rows if row['point'] == 'pre']
            if base:
                ax.axhline(statistics.median(base) + 20, color='#d62728', lw=0.8, ls='--', label='median pre + 20')
            ax.set_xticks(range(len(SSQ_POINTS)))
            ax.set_xticklabels(SSQ_POINTS)
            ax.set_ylabel('SSQ total')
            ax.legend(fontsize=7)
            fig.tight_layout()
            fig.savefig(os.path.join(out, 'fig_ssq.png'), dpi=150)
            plt.close(fig)
            figures.append('fig_ssq.png')
        report += ['## Figures', ''] + ['- {}'.format(f) for f in figures]
    except ImportError:
        report += ['(matplotlib missing: no figures)']

    with open(os.path.join(out, 'study_report.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(report) + '\n')
    print('Study results in', out)
    print('  study_report.md, table_*.csv, fig_*.png')
    return 0


if __name__ == '__main__':
    sys.exit(main())
