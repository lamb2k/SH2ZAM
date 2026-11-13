"""Verify the model-based adoption exhibit; no browser timing benchmark."""
from pathlib import Path
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from functools import partial
from threading import Thread
import hashlib
import json
import math
import os
import re
from playwright.sync_api import sync_playwright

root = Path(__file__).resolve().parent
assert hashlib.sha256((root / 'gainz.webp').read_bytes()).hexdigest() == 'bb0a8bb2f2520ddcb5baee7c9de7885e70af5d7a472bdabb9d1cb2e8ba677acc'


class QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, *_args):
        pass


server = ThreadingHTTPServer(('127.0.0.1', 0), partial(QuietHandler, directory=str(root)))
Thread(target=server.serve_forever, daemon=True).start()
url = f'http://127.0.0.1:{server.server_port}/index.html'


def wait_model(page):
    page.wait_for_function('window.SH2ZAM_MODEL_STATE?.ready === true')


def assert_comparison(page, scene, detail, baseline, kernel):
    state = page.evaluate('SH2ZAM_MODEL_STATE')
    record = page.evaluate("([scene,detail]) => SH2ZAM_MODEL_RESULTS.records.find(r => r.scene === scene && r.detail === detail)", [scene, detail])
    assert state['ready'], state
    assert (state['scene'], state['detail'], state['baseline'], state['kernel']) == (scene, detail, baseline, kernel), state
    before, after = record['medianCycles'][baseline], record['medianCycles'][kernel]
    assert state['vertices'] == record['vertices']
    assert state['referenceCycles'] == before and state['kernelCycles'] == after, state
    assert math.isclose(state['speedup'], before / after, rel_tol=1e-12), state
    assert math.isclose(state['savedPercent'], (1 - after / before) * 100, abs_tol=1e-10), state
    assert state['sourceCommit'] == page.evaluate('SH2ZAM_MODEL_RESULTS.run.sourceCommit')
    assert page.locator('#model-before-cycles').inner_text() != '—'
    assert page.locator('#model-after-cycles').inner_text() != '—'
    assert page.locator('#model-before-per-vertex').inner_text() != '—'
    assert page.locator('#model-after-per-vertex').inner_text() != '—'
    assert page.locator('#model-speedup').inner_text() != '—'
    assert f'detail {detail}' in page.locator('#model-workload').inner_text()
    assert record['fixedMismatches'] == 0
    assert record['maximumWorldError'] <= 2 / 65536
    return state


def assert_math_comparison(page, family, operation):
    state = page.evaluate('SH2ZAM_MATH_STATE')
    assert state['ready'] and state['family'] == family and state['operation'] == operation, state
    assert state['baselineCommit'] == '10786b64f4593d647d06d2a4a73e752c0c413dd0'
    assert state['sourceCommit'] == page.evaluate('SH2ZAM_MATH_MODEL.run.sourceCommit')
    assert len(state['records']) == 3
    expected = page.evaluate('(operation) => SH2ZAM_MATH_MODEL.records.filter(r => r.operation === operation)', operation)
    assert state['records'] == expected
    for row in state['records']:
        prefix = '#math-' + row['profile']
        before, after = row['cyclesPerCall']['before'], row['cyclesPerCall']['after']
        for suffix, cycles in [('before', before), ('after', after)]:
            rendered = float(page.locator(prefix + '-' + suffix).inner_text().replace(',', ''))
            assert math.isclose(rendered, cycles, abs_tol=0.00501), (row, suffix, rendered)
        assert math.isclose(row['speedup'], before / after, rel_tol=1e-12)
        assert math.isclose(row['cyclesSavedPercent'], (1 - after / before) * 100, abs_tol=1e-9)
        change = page.locator(prefix + '-change')
        if after > before:
            assert 'more' in change.inner_text() and 'math-loss' in change.get_attribute('class')
        elif after < before:
            assert 'less' in change.inner_text() and 'math-gain' in change.get_attribute('class')
        else:
            assert 'No change' in change.inner_text()
        assert page.locator(prefix + '-precision').inner_text() == 'Bit-exact'
        assert row['srSRetained'] and row['srSOutputMatch']
    assert 'bit-exact across all three profiles' in page.locator('#math-proof').inner_text()
    return state


def assert_workload_comparison(page, group, profile, size, baseline, kernel):
    state = page.evaluate('SH2ZAM_WORKLOAD_STATE')
    assert state['ready'], state
    assert (state['group'], state['profile'], state['size'], state['baselineChoice'], state['kernel']) == (group, profile, size, baseline, kernel), state
    record = page.evaluate('([group,profile,size]) => SH2ZAM_WORKLOAD_MODEL.records.find(r => r.group === group && r.profile === profile && r.size === size)', [group, profile, size])
    assert state['record'] == record
    if baseline == 'strongest':
        controls = ['scalar', 'composed', 'composed_mac'] if group == 'rotation' else ['api_brute', 'inline_brute', 'inline_scatter'] if group == 'spatial' else ['prior']
        selected = min(controls, key=lambda name: record['medianCycles'][name])
    else:
        selected = baseline
    assert state['baseline'] == selected
    before, after = record['medianCycles'][selected], record['medianCycles'][kernel]
    assert state['beforeCycles'] == before and state['afterCycles'] == after
    assert math.isclose(state['speedup'], before / after, rel_tol=1e-12)
    assert math.isclose(state['savedPercent'], (1 - after / before) * 100, abs_tol=1e-10)
    assert int(page.locator('#workload-before-cycles').inner_text().replace(',', '')) == before
    assert int(page.locator('#workload-after-cycles').inner_text().replace(',', '')) == after
    if after > before:
        assert 'EXTRA' in page.locator('#workload-savings-label').inner_text()
        assert 'control cost' in page.locator('#workload-speedup').inner_text()
    path_rows = page.locator('#workload-paths tr')
    assert path_rows.count() == len(record['paths'])
    for i, name in enumerate(record['paths']):
        assert int(path_rows.nth(i).locator('td').nth(0).inner_text().replace(',', '')) == record['medianCycles'][name]
    raw_details = page.locator('details.workload-details').nth(1)
    opened_for_check = raw_details.get_attribute('open') is None
    if opened_for_check:
        raw_details.locator('summary').click()
    try:
        raw_rows = page.locator('#workload-samples tr')
        assert raw_rows.count() == 7
        for i, sample in enumerate(record['samples']):
            checksum_label = raw_rows.nth(i).locator('th').inner_text()
            assert sample['inputChecksum'] in checksum_label, (record['id'], i, checksum_label, sample['inputChecksum'])
            cells = raw_rows.nth(i).locator('td')
            for j, name in enumerate(record['paths']):
                rendered = int(cells.nth(j).inner_text().replace(',', ''))
                assert rendered == sample['paths'][name]['cycles'], (record['id'], i, name, rendered, sample['paths'][name]['cycles'])
            checksum = cells.last.inner_text()
            assert checksum == sample['paths'][record['paths'][0]]['checksum'], (record['id'], i, checksum, sample['paths'][record['paths'][0]]['checksum'])
    finally:
        if opened_for_check:
            raw_details.locator('summary').click()
    assert record['setupIncluded'] and record['guards']['srSRetained'] and record['guards']['srSOutputMatch']
    assert 'Bit-exact' in page.locator('#workload-proof').inner_text()
    assert page.locator('#workload-all-cases tr').count() == 44
    return state


def export_workloads(page):
    details = page.locator('details.workload-details').nth(1)
    if details.get_attribute('open') is None:
        details.locator('summary').click()
    with page.expect_download() as result:
        page.click('#workload-export')
    download = result.value
    assert download.suggested_filename.startswith('sh2zam-workload-evidence-')
    target = root / 'test-result.json'
    download.save_as(target)
    data = json.loads(target.read_text())
    assert data == page.evaluate('SH2ZAM_WORKLOAD_MODEL'), 'Dedicated export must retain all 44 cases and raw evidence'
    assert len(data['records']) == 44 and data['mismatches'] == 0
    assert data['run']['setupIncluded'] and data['run']['hardwareMeasured'] is False
    assert data['run']['browserTimingUsed'] is False
    details.locator('summary').click()
    return data


def export_model(page):
    if not page.locator('#export').is_visible():
        page.locator('details.evidence > summary').click()
    with page.expect_download() as result:
        page.click('#export')
    download = result.value
    target = root / 'test-result.json'
    download.save_as(target)
    data = json.loads(target.read_text())
    state = page.evaluate('SH2ZAM_MODEL_STATE')
    assert data['schema'] == 'sh2zam-model-selection-v1'
    assert data['environment'] == 'isolated-sh2-interpreter'
    assert data['selection'] == {key: state[key] for key in ['scene', 'detail', 'baseline', 'kernel']}
    for key in ['referenceCycles', 'kernelCycles', 'savedPercent', 'speedup']:
        assert math.isclose(data['comparison'][key], state[key], abs_tol=1e-10)
    evidence = data['evidence']
    assert evidence['schema'] == 2
    assert evidence['kernelSet'] == 'float-plain-fixed-safe-prepared-fused-sh2-model-v2'
    assert len(evidence['records']) == 1
    assert evidence['records'][0]['scene'] == state['scene']
    assert evidence['records'][0]['detail'] == state['detail']
    assert evidence['run']['environment'] == 'isolated-sh2-interpreter'
    assert re.fullmatch('[a-f0-9]{40}', evidence['run']['sourceCommit'])
    assert re.fullmatch('[a-f0-9]{64}', evidence['run']['elfSHA256'])
    assert evidence['run']['evidenceUrl'].startswith('https://github.com/')
    assert len(evidence['run']['limitations']) >= 3
    assert evidence['guards']['fullRange']['candidateMask'] == 0
    assert evidence['guards']['saturationMode']['candidateMask'] == 5
    if page.evaluate('SH2ZAM_MATH_STATE.ready'):
        math_state = page.evaluate('SH2ZAM_MATH_STATE')
        assert data['mathLibrary']['selection'] == {key: math_state[key] for key in ['family', 'operation']}
        assert data['mathLibrary']['evidence'] == page.evaluate('SH2ZAM_MATH_MODEL')
        assert data['mathLibrary']['evidence']['comparison']['independentNonShazamBaseline'] is False
        assert data['mathLibrary']['evidence']['mismatches'] == 0
    else:
        assert 'mathLibrary' not in data
    if page.evaluate('SH2ZAM_WORKLOAD_STATE.ready'):
        workload_state = page.evaluate('SH2ZAM_WORKLOAD_STATE')
        selection = {key: workload_state[key] for key in ['group', 'profile', 'size', 'kernel']}
        selection['baseline'] = workload_state['baselineChoice']
        assert data['workloads']['selection'] == selection
        assert data['workloads']['evidence'] == page.evaluate('SH2ZAM_WORKLOAD_MODEL')
    else:
        assert 'workloads' not in data
    # Model provenance may contain its recorded capture date; this download must
    # not add browser benchmarking distributions or imply new target execution.
    serialized = json.dumps(data)
    for forbidden in ['samplesMs', 'medianMs', 'sampleIterations', 'minimumSampleDurationMs', 'userAgent']:
        assert f'"{forbidden}"' not in serialized
    return data


try:
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True, executable_path=os.environ.get('CHROMIUM_PATH'), args=['--no-sandbox'])
        page = browser.new_page(viewport={'width': 1440, 'height': 1500}, device_scale_factor=1)
        errors = []
        page.on('pageerror', lambda e: errors.append(str(e)))
        page.add_init_script("""
          window.__visualCalls = {frames: 0, before: 0, general: 0, after: 0};
          const raf = requestAnimationFrame;
          window.requestAnimationFrame = callback => raf.call(window, time => {
            __visualCalls.frames++;
            callback(time);
          });
          const instantiate = WebAssembly.instantiate;
          WebAssembly.instantiate = async (...args) => {
            const result = await instantiate(...args), exports = {...result.instance.exports};
            for (const name of ['before', 'general', 'after']) {
              const fn = exports[name];
              exports[name] = (...values) => { __visualCalls[name]++; return fn(...values); };
            }
            return {...result, instance: {exports}};
          };
        """)
        page.goto(url)
        wait_model(page)
        page.wait_for_function('window.SH2ZAM_EXHIBIT_STATE?.ready === true')
        assert page.evaluate('SH2ZAM_MODEL_RESULTS.schema') == 2
        assert len(page.evaluate('SH2ZAM_MODEL_RESULTS.records')) == 15
        assert page.evaluate('SH2ZAM_EXHIBIT_STATE.playing')
        assert_comparison(page, 'saturn', 3, 'plain', 'mac')
        assert 'optional' in page.locator('[data-kernel="mac"]').inner_text().lower()
        assert page.locator('#export').is_enabled()
        assert page.locator('input[type=range]').count() == 1
        assert page.locator('input[type=range]').get_attribute('id') == 'detail'
        for retired_id in ['before-time', 'general-time', 'after-time', 'batch-ratio', 'float-ratio', 'plan-time', 'kernel-time', 'benchmark', 'stage-before-time', 'stage-after-time']:
            assert page.locator('#' + retired_id).count() == 0, retired_id

        # Every math choice is backed by a separate recorded SH-2 fixture set.
        # This comparison upgrades the library; it is not the plain-C baseline.
        math_available = page.evaluate('!!window.SH2ZAM_MATH_MODEL')
        if math_available:
            assert page.evaluate('SH2ZAM_MATH_STATE.ready')
            assert_math_comparison(page, 'vector', 'vec3_normalize')
            operations = page.evaluate('[...new Map(SH2ZAM_MATH_MODEL.records.map(r => [r.operation, {family:r.family, operation:r.operation}])).values()]')
            assert len(operations) == 48
            for item in operations:
                page.select_option('#math-family', item['family'])
                page.select_option('#math-operation', item['operation'])
                assert_math_comparison(page, item['family'], item['operation'])
            page.select_option('#math-family', 'scalar')
            page.locator('#math-family').focus()
            page.keyboard.press('ArrowDown')
            assert page.evaluate('SH2ZAM_MATH_STATE.family') == 'vector'
            page.select_option('#math-operation', 'vec4_dot')
            page.locator('#math-operation').focus()
            page.keyboard.press('Home')
            page.keyboard.press('ArrowDown')
            assert_math_comparison(page, 'vector', 'vec3_dot')
            page.select_option('#math-operation', 'vec3_normalize')
            assert_math_comparison(page, 'vector', 'vec3_normalize')
            page.locator('.math-library').screenshot(path=str(root / 'screenshot-math-desktop.png'))
        else:
            assert not page.evaluate('SH2ZAM_MATH_STATE.ready')
            assert page.locator('#math-family').is_disabled()
            assert 'Awaiting' in page.locator('#math-status').inner_text()

        # Focus on the controls most likely to misstate the adoption gain.
        # The Node suite covers all 44 cases; these exercise the real DOM,
        # native select controls, displayed raw samples, and the actual download.
        workload_available = page.evaluate('!!window.SH2ZAM_WORKLOAD_MODEL')
        if workload_available:
            assert_workload_comparison(page, 'rotation', 'bounded', 512, 'strongest', 'mac')
            assert 'matrix + C batch' in page.locator('#workload-paths').inner_text()
            assert 'matrix + existing MAC.L' in page.locator('#workload-paths').inner_text()
            page.click('[data-workload-example="spatial"]')
            page.select_option('#workload-profile', 'crowded')
            page.select_option('#workload-size', '512')
            assert_workload_comparison(page, 'spatial', 'crowded', 512, 'strongest', 'index')
            assert 'Independent C · packed output' in page.locator('#workload-paths').inner_text()
            assert 'Independent C · sparse hit writes' in page.locator('#workload-paths').inner_text()
            assert page.locator('#workload-paths tr').count() == 4
            page.select_option('#workload-group', 'atan2')
            assert_workload_comparison(page, 'atan2', 'bounded', 32, 'prior', 'current')
            export_workloads(page)
            page.click('[data-workload-example="rotation"]')
            assert_workload_comparison(page, 'rotation', 'bounded', 512, 'strongest', 'mac')
            page.locator('.workload-lab').screenshot(path=str(root / 'screenshot-workload-desktop.png'))
        else:
            assert not page.evaluate('SH2ZAM_WORKLOAD_STATE.ready')
            assert page.locator('#workload-group').is_disabled()
            assert page.locator('#workload-before-cycles').inner_text() == '—'
            assert 'Awaiting' in page.locator('#workload-status').inner_text()

        # Target costs remain the same while the live visual advances.
        initial = page.evaluate('({model: SH2ZAM_MODEL_STATE, seconds: SH2ZAM_EXHIBIT_STATE.seconds, calls: __visualCalls})')
        page.wait_for_function('(seconds) => SH2ZAM_EXHIBIT_STATE.seconds > seconds + 0.1', arg=initial['seconds'])
        assert page.evaluate('SH2ZAM_MODEL_STATE') == initial['model']
        calls = page.evaluate('__visualCalls')
        frames = calls['frames'] - initial['calls']['frames']
        for kernel in ['before', 'general', 'after']:
            # Allow a resize-triggered draw; thousands of hidden benchmark calls
            # cannot be disguised as the single transform per displayed frame.
            assert 0 < calls[kernel] - initial['calls'][kernel] <= frames + 3, calls
        assert page.evaluate('SH2ZAM_EXHIBIT_STATE.fixedMismatches') == 0
        assert page.evaluate('SH2ZAM_EXHIBIT_STATE.outputsValid')
        page.click('#play')
        assert not page.evaluate('SH2ZAM_EXHIBIT_STATE.playing')

        # Every headline comes from its matching native workload, for both
        # starting points and both library API choices. A loss is not a failure.
        for scene in ['saturn', 'cubes', 'torus']:
            page.click(f'[data-scene="{scene}"]')
            for detail in range(1, 6):
                page.locator('#detail').fill(str(detail))
                visual = page.evaluate('SH2ZAM_EXHIBIT_STATE')
                assert visual['scene'] == scene and visual['detail'] == detail
                assert visual['count'] == page.evaluate('SH2ZAM_MODEL_STATE.vertices')
                assert visual['fixedMismatches'] == 0 and visual['outputsValid'], visual
                assert visual['maxWorldError'] < 2 / 65536, visual
                assert visual['maxError'] < 0.1, visual
                for baseline in ['plain', 'reference']:
                    page.click(f'[data-baseline="{baseline}"]')
                    for kernel in ['mac', 'prepared']:
                        page.click(f'[data-kernel="{kernel}"]')
                        assert_comparison(page, scene, detail, baseline, kernel)
            page.screenshot(path=str(root / f'screenshot-{scene}.png'), full_page=True)
        assert page.locator('#before-canvas').evaluate('el => getComputedStyle(el).clipPath') == 'inset(0px 50% 0px 0px)'

        page.click('[data-scene="saturn"]')
        page.locator('#detail').fill('3')
        page.locator('#detail').focus()
        page.keyboard.press('ArrowRight')
        assert page.evaluate('SH2ZAM_MODEL_STATE.detail') == 4
        page.click('[data-baseline="plain"]')
        page.click('[data-kernel="mac"]')
        exported = export_model(page)
        assert_comparison(page, 'saturn', 4, 'plain', 'mac')
        page.click('#play')
        assert page.evaluate('SH2ZAM_EXHIBIT_STATE.playing')
        assert page.locator('#export').is_enabled()
        page.screenshot(path=str(root / 'screenshot-desktop.png'), full_page=True)
        page.set_viewport_size({'width': 390, 'height': 844})
        page.screenshot(path=str(root / 'screenshot-mobile.png'), full_page=True)
        page.locator('.math-library').screenshot(path=str(root / 'screenshot-math-mobile.png'))
        page.locator('.workload-lab').screenshot(path=str(root / 'screenshot-workload-mobile.png'))
        assert page.evaluate('document.documentElement.scrollWidth <= innerWidth && document.body.scrollWidth <= innerWidth')
        assert not errors, errors

        context = browser.new_context(reduced_motion='reduce', viewport={'width': 800, 'height': 900})
        second = context.new_page()
        second.goto(url)
        wait_model(second)
        second.wait_for_function('window.SH2ZAM_EXHIBIT_STATE?.ready')
        assert not second.evaluate('SH2ZAM_EXHIBIT_STATE.playing')
        local = context.new_page()
        local.goto((root / 'index.html').as_uri())
        wait_model(local)
        local.wait_for_function('window.SH2ZAM_EXHIBIT_STATE?.ready')
        assert_comparison(local, 'saturn', 3, 'plain', 'mac')
        if math_available:
            assert_math_comparison(local, 'vector', 'vec3_normalize')

        # Missing math evidence is a pending capture, never a fabricated gain.
        missing_math = context.new_page()
        missing_math.route('**/math-results.js*', lambda route: route.fulfill(status=200, content_type='application/javascript', body='/* no capture */'))
        missing_math.goto(url)
        wait_model(missing_math)
        assert not missing_math.evaluate('SH2ZAM_MATH_STATE.ready')
        assert missing_math.locator('#math-family').is_disabled()
        assert missing_math.locator('#math-operation').is_disabled()
        for profile in ['bounded', 'tiny', 'full-range']:
            assert missing_math.locator('#math-' + profile + '-before').inner_text() == '—'
            assert missing_math.locator('#math-' + profile + '-after').inner_text() == '—'
        export_model(missing_math)
        missing_math.close()

        if math_available:
            # Presentation-only mutations test losses and rejected precision
            # evidence; neither becomes benchmark evidence or a committed file.
            math_loss = context.new_page()
            math_loss.add_init_script("""
                let data;
                Object.defineProperty(window, 'SH2ZAM_MATH_MODEL', {
                  configurable: true, get() { return data; },
                  set(value) {
                    data = value;
                    const row = data.records.find(r => r.operation === 'vec3_normalize' && r.profile === 'bounded');
                    for (const sample of row.samples) sample.after.cycles = sample.before.cycles * 2;
                    row.medianCycles.after = row.medianCycles.before * 2;
                    row.cyclesPerCall.after = row.cyclesPerCall.before * 2;
                    row.speedup = 0.5; row.cyclesSavedPercent = -100;
                  }
                });
            """)
            math_loss.goto(url)
            wait_model(math_loss)
            assert_math_comparison(math_loss, 'vector', 'vec3_normalize')
            assert '100.0% more' in math_loss.locator('#math-bounded-change').inner_text()
            math_loss.close()
            for mutation in [
                'data.records[0].srSOutputMatch = false;',
                'data.records.pop();',
                'data.records[0].samples.pop();',
                'data.records[0].samples[0].mismatches = 1;',
                'data.records[0].outputWords++; data.records[0].checkedOutputWords += 224;',
            ]:
                bad_math = context.new_page()
                bad_math.add_init_script("""
                    let data;
                    Object.defineProperty(window, 'SH2ZAM_MATH_MODEL', {
                      configurable: true, get() { return data; },
                      set(value) { data = value; MUTATION }
                    });
                """.replace('MUTATION', mutation))
                bad_math.goto(url)
                wait_model(bad_math)
                assert not bad_math.evaluate('SH2ZAM_MATH_STATE.ready')
                assert bad_math.locator('#math-family').is_disabled()
                assert 'did not pass' in bad_math.locator('#math-proof').inner_text()
                assert bad_math.locator('#math-bounded-before').inner_text() == '—'
                export_model(bad_math)
                bad_math.close()

        # The performance evidence has no WebAssembly dependency. Failure of
        # either feature detection or module loading must preserve model controls.
        for injection in [
            'Object.defineProperty(window, "WebAssembly", {value: undefined, configurable: true});',
            'WebAssembly.instantiate = async () => { throw new Error("Simulated unavailable visual module"); };',
        ]:
            unavailable = context.new_page()
            unavailable.add_init_script(injection)
            unavailable.goto(url)
            wait_model(unavailable)
            unavailable.wait_for_function('document.getElementById("failure")?.hidden === false')
            assert_comparison(unavailable, 'saturn', 3, 'plain', 'mac')
            if math_available:
                assert_math_comparison(unavailable, 'vector', 'vec3_normalize')
            unavailable.click('[data-scene="cubes"]')
            unavailable.locator('#detail').fill('5')
            unavailable.click('[data-baseline="reference"]')
            unavailable.click('[data-kernel="prepared"]')
            assert_comparison(unavailable, 'cubes', 5, 'reference', 'prepared')
            export_model(unavailable)
            unavailable.close()

        # A deliberately slower synthetic record tests presentation only; it is
        # never committed as benchmark evidence or used to claim a performance win.
        slower = context.new_page()
        slower.add_init_script("""
            let evidence;
            Object.defineProperty(window, 'SH2ZAM_MODEL_RESULTS', {
              configurable: true,
              get() { return evidence; },
              set(value) {
                evidence = value;
                const record = value.records.find(r => r.scene === 'saturn' && r.detail === 3);
                record.medianCycles.mac = record.medianCycles.plain * 2;
              }
            });
        """)
        slower.goto(url)
        wait_model(slower)
        assert slower.evaluate('SH2ZAM_MODEL_STATE.savedPercent') == -100
        assert slower.evaluate('SH2ZAM_MODEL_STATE.speedup') == 0.5
        savings = slower.locator('#model-savings').inner_text()
        label = slower.locator('#model-savings-label').inner_text().lower()
        assert '100' in savings and ('-' in savings or any(word in label for word in ['extra', 'more', 'increase', 'slower'])), (savings, label)
        slower.close()

        # Live output corruption must be reported as a visual check failure, and
        # must not rewrite independent, previously recorded native model results.
        invalid = context.new_page()
        invalid.add_init_script("""
          const instantiate = WebAssembly.instantiate;
          WebAssembly.instantiate = async (...args) => {
            const result = await instantiate(...args), exports = {...result.instance.exports};
            const before = exports.before;
            exports.before = () => {
              before();
              new Float32Array(exports.memory.buffer, exports.before_ptr(), 1)[0] = NaN;
            };
            return {...result, instance: {exports}};
          };
        """)
        invalid.goto(url)
        wait_model(invalid)
        invalid.wait_for_function('window.SH2ZAM_EXHIBIT_STATE?.outputChecksums?.reference')
        assert invalid.evaluate('SH2ZAM_EXHIBIT_STATE.fixedMismatches') == 0
        assert invalid.evaluate('SH2ZAM_EXHIBIT_STATE.outputsValid') is False
        assert_comparison(invalid, 'saturn', 3, 'plain', 'mac')
        assert invalid.locator('#export').is_enabled()
        invalid.close()

        print('Model selection:', json.dumps(exported['selection']), json.dumps(exported['comparison']), flush=True)
        browser.close()
finally:
    server.shutdown()

print('PASS: 15 native transform workloads, optional 48-operation math capture, focused workload controls/raw samples/full export, honest losses, exactness guards, keyboard/mobile, offline HTML, and WebAssembly-independent evidence')
