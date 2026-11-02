"""
Where the time of each simulation step goes (synchronous mode: the next step only starts once all of
them are done, so their total limits the steps per second, hence the frame rate the run keeps up with).

    with step_timing.measure('agent'):
        ...

Parts measured (ms per step):
    server     world.tick(): the CARLA server simulating and rendering the step (VR included)
    provider   CarlaDataProvider.on_carla_tick(): the actors' positions / velocities read from the server
    sensors    the agent waiting for its sensor data of the step
    planner    the driving agent (BasicAgent2.run_step: planning, hazards, dashboard commands)
    logger     the session logger (frames.csv row, hazards, TTCs)
    scenario   the scenario tree (behaviors and criteria)
    other      the rest of the step (control applied, prints, ...)

Every REPORT_PERIOD seconds a line is printed; the whole run's means go into summary.json (step_timing_ms).
"""

import contextlib
import time

REPORT_PERIOD = 10.0  # [s] wall clock

PARTS = ('server', 'provider', 'sensors', 'planner', 'logger', 'scenario', 'other')

_total = {}
_window = {}
_steps = 0
_window_steps = 0
_step_start = None
_window_start = None


@contextlib.contextmanager
def measure(part):
    start = time.perf_counter()
    try:
        yield
    finally:
        elapsed = time.perf_counter() - start
        _total[part] = _total.get(part, 0.0) + elapsed
        _window[part] = _window.get(part, 0.0) + elapsed


def step_begin():
    global _step_start, _window_start
    _step_start = time.perf_counter()
    if _window_start is None:
        _window_start = _step_start


def step_end():
    """Ends a step: what wasn't measured is 'other'; prints the report every REPORT_PERIOD seconds"""
    global _steps, _window_steps, _window_start
    if _step_start is None:
        return
    now = time.perf_counter()
    step = now - _step_start
    _steps += 1
    _window_steps += 1
    _total['step'] = _total.get('step', 0.0) + step
    _window['step'] = _window.get('step', 0.0) + step
    if now - _window_start >= REPORT_PERIOD and _window_steps > 0:
        print('Step timing: ' + _format(_window, _window_steps, now - _window_start))
        _window.clear()
        _window_steps = 0
        _window_start = now


def _means(values, steps):
    means = {p: 1000.0 * values.get(p, 0.0) / steps for p in PARTS if p != 'other'}
    step = 1000.0 * values.get('step', 0.0) / steps
    means['other'] = max(0.0, step - sum(means.values()))
    means['step'] = step
    return means


def _format(values, steps, wall):
    means = _means(values, steps)
    parts = ', '.join('{} {:.1f}'.format(p, means[p]) for p in PARTS)
    return '{:.1f} ms per step ({}) -> {:.1f} steps/s'.format(means['step'], parts, steps / wall if wall > 0 else 0.0)


def summary():
    """The whole run's mean ms per step and part (None before any step)"""
    if _steps == 0:
        return None
    return {p: round(v, 2) for p, v in _means(_total, _steps).items()}


def reset():
    global _steps, _window_steps, _step_start, _window_start
    _total.clear()
    _window.clear()
    _steps = _window_steps = 0
    _step_start = _window_start = None
