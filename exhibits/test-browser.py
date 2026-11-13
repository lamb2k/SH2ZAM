"""Local or CI browser smoke test; requires Playwright + Chromium."""
from pathlib import Path
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from functools import partial
from threading import Thread
import os
from playwright.sync_api import sync_playwright
root=Path(__file__).resolve().parent
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
    page.click('[data-scene=saturn]')
    page.locator('#detail').fill('3')
    page.locator('#wipe').fill('50')
    page.locator('#time').fill('1.35')
    page.click('#benchmark')
    page.wait_for_function('document.getElementById("export").disabled === false')
    with page.expect_download() as result:
        page.click('#export')
    result.value.save_as(root/'test-result.json')
    page.screenshot(path=str(root/'screenshot-desktop.png'),full_page=True)
    page.set_viewport_size({'width':390,'height':844})
    page.screenshot(path=str(root/'screenshot-mobile.png'),full_page=True)
    assert page.evaluate('document.documentElement.scrollWidth <= innerWidth')
    assert not errors,errors
    context=browser.new_context(reduced_motion='reduce',viewport={'width':800,'height':900})
    second=context.new_page(); second.goto(url)
    second.wait_for_function('window.SH2ZAM_EXHIBIT_STATE?.ready')
    assert second.evaluate('SH2ZAM_EXHIBIT_STATE.playing') is False
    browser.close()
server.shutdown()
print('PASS: scenes, high detail, pause, scrub, reveal, benchmark/export, mobile, reduced motion')
