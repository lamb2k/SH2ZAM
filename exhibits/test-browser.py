"""Local or CI browser smoke test; requires Playwright + Chromium."""
from pathlib import Path
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from functools import partial
from threading import Thread
import hashlib
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
    assert page.locator('#after-time').inner_text() != '—'
    page.click('#play')
    assert not page.evaluate('SH2ZAM_EXHIBIT_STATE.playing')
    for scene in ['saturn','cubes','torus']:
        page.click(f'[data-scene={scene}]')
        page.locator('#detail').fill('5')
        page.locator('#time').fill('3.15')
        state=page.evaluate('SH2ZAM_EXHIBIT_STATE')
        assert state['scene']==scene and state['count']<16384
        assert state['maxError']<0.1,state
        page.locator('#wipe').fill('25')
        assert page.locator('#before-canvas').evaluate('el=>el.style.clipPath')=='inset(0px 75% 0px 0px)'
        page.screenshot(path=str(root/f'screenshot-{scene}.png'),full_page=True)
    page.click('[data-scene=saturn]')
    page.locator('#detail').fill('3')
    page.locator('#wipe').fill('50')
    page.locator('#time').fill('1.35')
    # Pointer capture on the draggable divider and native keyboard control.
    handle=page.locator('#drag').bounding_box()
    stage=page.locator('#stage').bounding_box()
    page.mouse.move(handle['x']+handle['width']/2,handle['y']+handle['height']/2)
    page.mouse.down()
    page.mouse.move(stage['x']+stage['width']*.7,handle['y']+handle['height']/2,steps=8)
    page.mouse.up()
    assert abs(float(page.locator('#wipe').input_value())-70)<1
    page.locator('#wipe').focus()
    page.keyboard.press('ArrowRight')
    assert float(page.locator('#wipe').input_value())>70
    page.locator('#wipe').fill('50')
    page.click('#benchmark')
    page.wait_for_function('document.getElementById("export").disabled === false')
    with page.expect_download() as result:
        page.click('#export')
    result.value.save_as(root/'test-result.json')
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
    browser.close()
server.shutdown()
print('PASS: exact artwork, scenes, high detail, pause, scrub, reveal, pointer/keyboard, benchmark/export, mobile, reduced motion, offline HTML')
