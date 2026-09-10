#!/usr/bin/env python3
"""Capture the native desktop client; optionally exercise its actual UI controls.

Default mode is read-only. --exercise requires a connected IDLE circle and
leaves the completed image formation visible. On exercise failure, it clicks
Stop when the interface is available. Run separately from other live tests.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import time
from urllib.parse import urlsplit

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

# The application keeps its state private to its ES module. Read the same
# measured endpoint independently; all mutations still go through DOM controls.
INSTALL_OBSERVER = """
(() => {
  if (window.__desktopProbe) return;
  const probe = window.__desktopProbe = {state: null, error: null, updated: 0};
  async function poll() {
    try {
      const response = await fetch('/api/state', {cache: 'no-store', signal: AbortSignal.timeout(2000)});
      if (!response.ok) throw new Error(`State request returned ${response.status}`);
      probe.state = await response.json(); probe.error = null; probe.updated = Date.now();
    } catch (error) { probe.error = error.message; }
    probe.timer = setTimeout(poll, 250);
  }
  poll();
})();
"""

READ_DOM = """
JSON.stringify((() => {
  const element = id => document.getElementById(id);
  const loaded = id => { const image = element(id); return Boolean(image && !image.hidden && image.complete && image.naturalWidth > 0); };
  return {
    title: document.title, canvas_count: document.querySelectorAll('canvas').length,
    render_fps: Number(element('render-rate')?.textContent || 0),
    connected: Boolean(element('connection')?.textContent.includes('Gazebo connected')),
    simulation_state: element('simulation-state')?.textContent || '',
    simulation_error: Boolean(element('simulation-state')?.classList.contains('error')),
    text: document.body?.innerText || '', state: window.__desktopProbe?.state || null,
    state_age_s: window.__desktopProbe?.updated ? (Date.now() - window.__desktopProbe.updated) / 1000 : null,
    state_error: window.__desktopProbe?.error || null,
    ui: {
      enabled: Object.fromEntries(['start-show','use-sample','apply-formation','apply-settings','stop-show'].map(id => [id, Boolean(element(id) && !element(id).disabled)])),
      source_loaded: loaded('source-preview'), points_loaded: loaded('points-preview'),
      formation: element('formation')?.value, color_mode: element('color_mode')?.value,
      settings_dirty: Boolean(element('settings-dirty') && !element('settings-dirty').hidden),
      notice_error: Boolean(element('notice') && !element('notice').hidden && element('notice').classList.contains('error')),
      notice: element('notice-text')?.textContent || '',
      selected_view: document.querySelector('.view-controls button.selected')?.id || null,
      image_metadata: element('image-metadata')?.textContent || ''
    }
  };
})())
"""


def click(button):
    return f"""const button = document.getElementById({json.dumps(button)});
if (!button || button.disabled) throw new Error('UI button is missing or disabled');
button.click();"""


def finite_rows(values, count):
    return isinstance(values, list) and len(values) == count and all(
        isinstance(row, list) and len(row) == 3 and all(
            isinstance(value, (int, float)) and math.isfinite(value) for value in row
        ) for row in values
    )


def target_error(state):
    count = state.get('count', 0)
    actual, targets = state.get('actual_positions'), state.get('targets')
    if not isinstance(count, int) or not 1 <= count <= 400 or not finite_rows(actual, count) or not finite_rows(targets, count):
        return None
    return max(math.dist(position, target) for position, target in zip(actual, targets))


def runtime_failure(data):
    """A healthy renderer cannot make an errored simulation pass capture."""
    state = data.get('state') or {}
    if (state.get('state') == 'ERROR' or data.get('simulation_error')
            or str(data.get('simulation_state', '')).strip().upper() == 'ERROR'):
        return 'Runtime entered ERROR: ' + str(state.get('error') or 'desktop simulation status reports ERROR')
    return None


class DesktopExercise:
    """A bounded workflow driven by measured state and visible DOM results."""

    def __init__(self, stage_timeout):
        self.stage_timeout = stage_timeout
        self.stage = 'ready'
        self.stage_started = time.monotonic()
        self.checks = []
        self.initial_positions = None
        self.motion_seen = False
        self.started_show = False
        self.completed = False

    def mark(self, name, **details):
        self.checks.append({'name': name, 'passed': True, **details})
        print('PASS:', name, flush=True)

    def move_to(self, stage):
        self.stage = stage
        self.stage_started = time.monotonic()

    @staticmethod
    def complete(state, formation):
        error = target_error(state)
        return (state.get('formation') == formation and state.get('state') in {'FORMATION_COMPLETE', 'ANIMATING'}
                and state.get('progress', 0) >= .99 and not state.get('pending_formation')
                and error is not None and error <= .35)

    def advance(self, data):
        if time.monotonic() - self.stage_started > self.stage_timeout:
            raise TimeoutError(f"Desktop workflow timed out during {self.stage}")
        if runtime_failure(data):
            raise AssertionError(runtime_failure(data))
        state, ui = data.get('state'), data.get('ui', {})
        if not state:
            return None
        if ui.get('notice_error'):
            raise AssertionError(f"UI rejected an action: {ui.get('notice')}")
        fresh = data.get('state_age_s') is not None and data['state_age_s'] < 2
        if not fresh or not state.get('connected') or not data.get('connected') or state.get('layout_pending'):
            return None
        enabled = ui.get('enabled', {})
        if self.stage == 'ready':
            if data.get('canvas_count', 0) < 1 or not (data.get('render_fps') or 0) > 0 or not enabled.get('start-show'):
                return None
            if state.get('state') != 'IDLE' or state.get('formation') != 'circle':
                raise AssertionError('Start --exercise with an IDLE circle (after API acceptance or a fresh launch)')
            if state.get('pending_config') or state.get('pending_formation') or state.get('image_processing'):
                return None
            count = state.get('count')
            if not finite_rows(state.get('actual_positions'), count):
                raise AssertionError('Initial actual Gazebo positions are incomplete or nonfinite')
            self.initial_positions = state['actual_positions']
            self.mark('native WebGL view connected to measured Gazebo poses', count=count)
            self.started_show = True
            self.move_to('circle')
            return click('start-show')
        if self.stage == 'circle':
            if state.get('state') in {'TAKEOFF', 'TRANSITIONING'} and finite_rows(state.get('actual_positions'), len(self.initial_positions)):
                displacement = max(math.dist(a, b) for a, b in zip(state['actual_positions'], self.initial_positions))
                self.motion_seen = self.motion_seen or displacement > .2
            if self.complete(state, 'circle') and enabled.get('use-sample'):
                if not self.motion_seen:
                    raise AssertionError('Circle completed without observing measured takeoff or transition motion')
                self.mark('Start button completes measured circle', target_error_m=target_error(state))
                self.move_to('image_processing')
                return click('use-sample')
        elif self.stage == 'image_processing':
            if (state.get('image') and not state.get('image_processing') and ui.get('source_loaded')
                    and ui.get('points_loaded') and ui.get('formation') == 'image' and enabled.get('apply-formation')):
                self.mark('Use sample processes image and displays both previews', metadata=ui.get('image_metadata'))
                self.move_to('image_formation')
                return click('apply-formation')
        elif self.stage == 'image_formation':
            if self.complete(state, 'image') and enabled.get('apply-settings'):
                self.mark('Apply formation reaches measured image targets', target_error_m=target_error(state))
                self.move_to('image_colors')
                return """const select = document.getElementById('color_mode');
if (!select) throw new Error('Image color selector is missing');
select.value = 'image';
select.dispatchEvent(new Event('input', {bubbles: true}));
select.dispatchEvent(new Event('change', {bubbles: true}));
""" + click('apply-settings')
        elif self.stage == 'image_colors':
            if (state.get('config', {}).get('color_mode') == 'image' and ui.get('color_mode') == 'image'
                    and not ui.get('settings_dirty') and enabled.get('apply-settings') and self.complete(state, 'image')):
                colors = state.get('colors')
                if not finite_rows(colors, state['count']):
                    raise AssertionError('Image lighting colors are incomplete or nonfinite')
                distinct = len({tuple(round(value, 3) for value in color) for color in colors})
                if distinct <= 3:
                    raise AssertionError(f'Image lighting contains only {distinct} distinct RGB colors')
                self.mark('Image color setting applies through the UI', distinct_commanded_colors=distinct)
                self.move_to('settle')
                return """for (const id of ['view-front', 'view-fit']) {
  const button = document.getElementById(id);
  if (!button || button.disabled) throw new Error('Camera view button unavailable');
  button.click();
}
document.querySelector('.design-panel')?.scrollTo(0, 0);
"""
        elif self.stage == 'settle' and time.monotonic() - self.stage_started >= 7:
            if ui.get('selected_view') != 'view-fit' or not self.complete(state, 'image'):
                raise AssertionError('Final camera or measured image formation changed before capture')
            self.mark('Front and Fit controls frame the completed image')
            self.completed = True
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8765')
    parser.add_argument('--seconds', type=int, default=12, help='Read-only capture delay')
    parser.add_argument('--output', default='artifacts/desktop.png')
    parser.add_argument('--exercise', action='store_true', help='Click the full Start-to-image workflow; requires IDLE circle')
    parser.add_argument('--timeout', type=float, default=420, help='Overall exercise wall-time limit')
    parser.add_argument('--stage-timeout', type=float, default=180, help='Per-stage exercise wall-time limit')
    args = parser.parse_args()
    parsed = urlsplit(args.url)
    if parsed.scheme != 'http' or parsed.hostname not in {'127.0.0.1', 'localhost', '::1'} or parsed.username or parsed.password:
        parser.error('--url must be an HTTP address on localhost')
    if not all(math.isfinite(value) and value > 0 for value in (args.timeout, args.stage_timeout)):
        parser.error('Timeouts must be positive and finite')

    from PySide6.QtCore import QTimer
    from PySide6.QtWebEngineCore import QWebEnginePage
    from PySide6.QtWidgets import QApplication
    from drone_swarm.gui.main_window import MainWindow

    messages = []
    class Page(QWebEnginePage):
        def javaScriptConsoleMessage(self, level, message, line, source):
            messages.append({'level': str(level), 'message': message, 'line': line, 'source': source})
            print('WEB:', message, flush=True)

    app = QApplication([sys.argv[0]])
    window = MainWindow(args.url)
    # Automated failures must produce an artifact rather than a modal Retry box.
    window.view.loadFinished.disconnect(window._loaded)
    page = Page(window.view)
    window.view.setPage(page)
    window.show()
    exercise = DesktopExercise(args.stage_timeout) if args.exercise else None
    created = time.monotonic()
    outcome = {'passed': False, 'finished': False, 'finishing': False, 'pending': False, 'last': {}}
    poll_timer = QTimer()

    def finish(data=None, failure=None):
        if outcome['finished']:
            return
        outcome['finished'] = True
        poll_timer.stop()
        data = dict(data or outcome['last'])
        failure = failure or runtime_failure(data)
        state = data.pop('state', None)
        data.update(console=messages, exercise=bool(exercise), wall_seconds=time.monotonic() - created)
        data['checks'] = [] if exercise is None else exercise.checks
        data['failure'] = failure
        if state:
            data['measured_state'] = {key: state.get(key) for key in (
                'state', 'formation', 'count', 'actual_count', 'connected', 'progress',
                'actual_simulation_time', 'pose_sequence', 'tracking_error_m', 'actual_min_separation_m',
                'control_update_hz', 'pose_update_hz', 'real_time_factor', 'colors_source')}
            data['measured_state']['target_error_m'] = target_error(state)
        errors = [message for message in messages if 'Error' in message['level']]
        data['passed'] = bool(not failure and data.get('canvas_count', 0) > 0 and (data.get('render_fps') or 0) > 0
                              and data.get('connected') and not errors and (exercise is None or exercise.completed))
        target = Path(args.output).expanduser()
        try:
            target.parent.mkdir(parents=True, exist_ok=True)
            if not window.grab().save(str(target)):
                raise RuntimeError('Qt could not save the desktop frame')
            target.with_suffix('.json').write_text(json.dumps(data, indent=2, allow_nan=False) + '\n', encoding='utf-8')
            outcome['passed'] = data['passed']
        except Exception as error:
            outcome['passed'] = False
            print(f'FAIL: writing desktop artifact: {error}', file=sys.stderr, flush=True)
        print(f"{'PASS' if outcome['passed'] else 'FAIL'}: desktop {'workflow' if exercise else 'capture'}; output={target}; failure={failure}", flush=True)
        window.close()
        app.quit()

    def fail(error):
        if outcome['finished'] or outcome['finishing']:
            return
        outcome['finishing'] = True
        poll_timer.stop()
        reason = f'{type(error).__name__}: {error}' if isinstance(error, BaseException) else str(error)
        if exercise is not None and exercise.started_show:
            page.runJavaScript("document.getElementById('stop-show')?.click();")
            QTimer.singleShot(600, lambda: finish(failure=reason))
        else:
            finish(failure=reason)

    def action_finished(payload):
        outcome['pending'] = False
        if outcome['finished'] or outcome['finishing']:
            return
        try:
            result = json.loads(payload)
            if result.get('error'):
                raise AssertionError(result['error'])
        except Exception as error:
            fail(error)

    def received(payload):
        outcome['pending'] = False
        if outcome['finished'] or outcome['finishing']:
            return
        try:
            data = json.loads(payload)
            outcome['last'] = data
            if exercise is None:
                finish(data)
                return
            command = exercise.advance(data)
            if command:
                outcome['pending'] = True
                wrapped = "JSON.stringify((() => {try {" + command + ";return {ok:true};} catch(error) {return {error:error.message};}})())"
                page.runJavaScript(wrapped, action_finished)
            elif exercise.completed:
                finish(data)
        except Exception as error:
            fail(error)

    def capture():
        if not outcome['pending'] and not outcome['finished'] and not outcome['finishing']:
            outcome['pending'] = True
            page.runJavaScript(READ_DOM, received)

    def loaded(succeeded):
        if not succeeded:
            fail('Desktop page failed to load')
            return
        page.runJavaScript(INSTALL_OBSERVER)
        if exercise is not None:
            poll_timer.start(250)

    window.view.loadFinished.connect(loaded)
    poll_timer.timeout.connect(capture)
    window.reload()
    if exercise is None:
        QTimer.singleShot(max(3, args.seconds) * 1000, capture)
    limit = args.timeout if exercise is not None else max(3, args.seconds) + 10
    QTimer.singleShot(math.ceil(limit * 1000), lambda: fail('Desktop test exceeded its overall wall-time limit'))
    QTimer.singleShot(math.ceil((limit + 5) * 1000), lambda: finish(failure='Desktop test emergency timeout'))
    app.exec()
    return 0 if outcome['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
