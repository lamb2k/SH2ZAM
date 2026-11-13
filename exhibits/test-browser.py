"""Local or CI browser smoke test; requires Playwright + Chromium."""
from pathlib import Path
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from functools import partial
from threading import Thread
import hashlib
import json
import os
from playwright.sync_api import sync_playwright
root=Path(__file__).resolve().parent
# Catch upload corruption even when a permissive decoder still displays it.
assert hashlib.sha256((root/'gainz.webp').read_bytes()).hexdigest() == 'bb0a8bb2f2520ddcb5baee7c9de7885e70af5d7a472bdabb9d1cb2e8ba677acc'
server=ThreadingHTTPServer(('127.0.0.1',0), partial(SimpleHTTPRequestHandler,directory=str(root)))
Thread(target=server.serve_forever,daemon=True).start()
url=f'http://127.0.0.1:{server.server_port}/index.html'
with sync_playwright() as p:
    browser=p.chromium.launch(headless=True, executable_path=os.environ.get('CHROMIUM_PATH'), args=['--no-sandbox'])
    page=browser.new_page(viewport={'width':1440,'height':1500},device_scale_factor=1)
    errors=[]
    page.on('pageerror',lambda e: errors.append(str(e)))
    page.goto(url)
    page.wait_for_function('window.SH2ZAM_EXHIBIT_STATE?.ready === true')
    page.wait_for_function('Number(document.getElementById("before-time").dataset.samples) >= 2')
    assert page.evaluate('SH2ZAM_EXHIBIT_STATE.playing')
    assert page.locator('#before-time').inner_text() != '—'
    assert page.locator('#general-time').inner_text() != '—'
    assert page.locator('#after-time').inner_text() != '—'
    assert page.locator('#batch-ratio').inner_text() != '—'
    assert page.locator('#stage-before-time').inner_text()==page.locator('#before-time').inner_text()
    assert page.locator('#stage-after-time').inner_text()==page.locator('#after-time').inner_text()
    assert 'LIVE' in page.locator('#live-status').inner_text()
    assert page.locator('#saturn-status').inner_text()=='Not measured'
    assert page.locator('#saturn-reference').inner_text()=='—'
    assert page.locator('#saturn-general').inner_text()=='—'
    assert page.locator('#saturn-optimized').inner_text()=='—'
    assert page.locator('#model-status').inner_text()=='Recorded · guest cycles'
    assert page.locator('#model-reference').inner_text()!='—'
    assert page.locator('#model-mac-ratio').inner_text()!='—'
    assert 'Physical-console timings remain unmeasured' in page.locator('.model-recorded').inner_text()
    assert page.locator('.kernel-table pre code').count()==3
    assert 'float result' in page.locator('.kernel-table').inner_text()
    assert 'sh2zi_clamp(' in page.locator('.kernel-table').inner_text()
    assert 'sh2zi_floor16(sum)' in page.locator('.kernel-table').inner_text()
    assert 'sh2zi_prepared_floor16(sum)' in page.locator('.kernel-table').inner_text()
    assert page.evaluate('SH2ZAM_EXHIBIT_STATE.preparedFastPath') is True
    assert page.evaluate('SH2ZAM_EXHIBIT_STATE.fixedMismatches')==0
    assert 'bit-exact' in page.locator('#fixed-equality').inner_text()
    page.click('#play')
    assert not page.evaluate('SH2ZAM_EXHIBIT_STATE.playing')
    for scene in ['saturn','cubes','torus']:
        page.click(f'[data-scene={scene}]')
        page.locator('#detail').fill('5')
        state=page.evaluate('SH2ZAM_EXHIBIT_STATE')
        assert state['scene']==scene and state['count']<16384
        assert f"detail 5 · {state['count']} vertices" in page.locator('#saturn-workload').inner_text()
        assert f"detail 5 · {state['count']} vertices" in page.locator('#model-workload').inner_text()
        assert state['optimizedStrategy']==('coordinate-reuse' if scene=='cubes' else 'prepared-scalar')
        assert state['maxError']<0.1,state
        assert state['fixedMismatches']==0,state
        assert state['maxWorldError']<2/65536,state
        assert page.locator('#before-canvas').evaluate('el=>getComputedStyle(el).clipPath')=='inset(0px 50% 0px 0px)'
        page.screenshot(path=str(root/f'screenshot-{scene}.png'),full_page=True)
    page.click('[data-scene=saturn]')
    page.locator('#detail').fill('3')
    assert page.locator('input[type=range]').count()==1
    assert page.locator('input[type=range]').get_attribute('id')=='detail'
    page.locator('#detail').focus()
    page.keyboard.press('ArrowRight')
    assert page.evaluate('SH2ZAM_EXHIBIT_STATE.detail')==4
    page.click('#benchmark')
    page.wait_for_function('document.getElementById("export").disabled === false')
    with page.expect_download() as result:
        page.click('#export')
    result.value.save_as(root/'test-result.json')
    data=json.loads((root/'test-result.json').read_text())
    assert data['schema']==4 and data['kernelSet']=='float-safe-mesh-v1'
    assert data['optimizedStrategy']=='prepared-scalar'
    assert data['preparedFastPath'] is True
    assert data['minimumSampleDurationMs']==5
    assert data['fixedMismatches']==0 and data['saturnBenchmark'] is False
    assert data['outputChecksums']['general']==data['outputChecksums']['optimized']
    assert all(len(value)==8 for value in data['outputChecksums'].values())
    for path in ['reference','general','optimized']:
        assert len(data['samplesMs'][path])==21
        assert data['medianMs'][path]>=0
        assert len(data['sampleIterations'][path])==21
        assert all(value>=5 for value in data['sampleElapsedMs'][path])
    for component in ['planPreparation','kernelOnly']:
        assert len(data['components']['samples'][component])==21
        assert data['components']['medianMs'][component]>0
        assert all(value['elapsedMs']>=5 for value in data['components']['samples'][component])
    assert 'matrix qualification and product-table rebuilding per optimized batch' in data['includes']
    assert data['maximumWorldError']['general']==data['maximumWorldError']['optimized']
    print('Browser capture:',json.dumps({key:data[key] for key in ['schema','scene','detail','vertices','medianMs','components','ratios','maximumWorldError','fixedMismatches','outputChecksums','wasmSHA256']}),flush=True)
    # Check the actual adaptive strategy and full inclusive timing in every
    # scene. A speedup is reported by measurements, never a pass/fail condition.
    for extra_scene in ['cubes','torus']:
        page.click(f'[data-scene={extra_scene}]')
        page.locator('#detail').fill('4')
        page.click('#benchmark')
        page.wait_for_function('document.getElementById("export").disabled === false')
        with page.expect_download() as extra_result:
            page.click('#export')
        target=root/f'test-result-{extra_scene}.json'
        extra_result.value.save_as(target)
        extra=json.loads(target.read_text())
        assert extra['schema']==4 and extra['kernelSet']=='float-safe-mesh-v1'
        assert extra['optimizedStrategy']==('coordinate-reuse' if extra_scene=='cubes' else 'prepared-scalar')
        assert extra['fixedMismatches']==0
        assert extra['outputChecksums']['general']==extra['outputChecksums']['optimized']
        assert all(value>=5 for path in extra['sampleElapsedMs'].values() for value in path)
        assert extra['maximumWorldError']['optimized']<2/65536
        print('Additional browser capture:',json.dumps({key:extra[key] for key in ['scene','detail','vertices','optimizedStrategy','medianMs','ratios','fixedMismatches','maximumWorldError','wasmSHA256']}),flush=True)
    captured=page.locator('#before-time').inner_text()
    page.wait_for_timeout(650)
    assert page.locator('#before-time').inner_text()==captured
    samples=int(page.locator('#before-time').get_attribute('data-samples'))
    page.click('#play')
    page.wait_for_function('(n)=>Number(document.getElementById("before-time").dataset.samples)>n',arg=samples)
    assert page.locator('#export').is_disabled()
    page.screenshot(path=str(root/'screenshot-desktop.png'),full_page=True)
    page.set_viewport_size({'width':390,'height':844})
    page.screenshot(path=str(root/'screenshot-mobile.png'),full_page=True)
    assert page.evaluate('document.documentElement.scrollWidth <= innerWidth')
    assert not errors,errors
    context=browser.new_context(reduced_motion='reduce',viewport={'width':800,'height':900})
    second=context.new_page(); second.goto(url)
    second.wait_for_function('window.SH2ZAM_EXHIBIT_STATE?.ready')
    assert second.evaluate('SH2ZAM_EXHIBIT_STATE.playing') is False
    # Downloaded artifact should also run without a web server.
    local=context.new_page(); local.goto((root/'index.html').as_uri())
    local.wait_for_function('window.SH2ZAM_EXHIBIT_STATE?.ready')
    # A corrupt float result must not export timings, even when fixed paths agree.
    invalid=context.new_page()
    invalid.add_init_script('''
      const instantiate=WebAssembly.instantiate;
      WebAssembly.instantiate=async(...args)=>{
        const result=await instantiate(...args),exports={...result.instance.exports};
        const before=exports.before;
        exports.before=()=>{before();new Float32Array(exports.memory.buffer,exports.before_ptr(),1)[0]=NaN;};
        return {...result,instance:{exports}};
      };
    ''')
    invalid.goto(url)
    invalid.wait_for_function('window.SH2ZAM_EXHIBIT_STATE?.ready')
    assert invalid.evaluate('SH2ZAM_EXHIBIT_STATE.fixedMismatches')==0
    assert invalid.evaluate('SH2ZAM_EXHIBIT_STATE.outputsValid') is False
    invalid.click('#benchmark')
    assert invalid.locator('#export').is_disabled()
    assert invalid.locator('#batch-ratio').inner_text()=='—'
    assert 'precision checks' in invalid.locator('#result').inner_text()
    browser.close()
server.shutdown()
print('PASS: adaptive exact strategies, inclusive table rebuilding, all-scene capture, isolated SH-2 model labels, live paths, artwork, controls, export, mobile, reduced motion, offline HTML')

