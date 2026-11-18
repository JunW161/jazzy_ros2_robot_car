"""Browser verification against an explicitly OFFLINE Web instance.

Run with an ephemeral account and data directory, never a live robot server:
  python3 tests/ui_smoke.py --url http://127.0.0.1:8098 \
      --username ui_review --password-file /path/to/temporary/password
Requires Playwright and Chromium for development only.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import shutil
from playwright.sync_api import sync_playwright


def run(url, username, password, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    errors = []
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(
            executable_path=shutil.which('chromium'), headless=True,
            args=['--no-sandbox'],
        )
        page = browser.new_page(viewport={'width': 1440, 'height': 1000}, device_scale_factor=1)
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.goto(url, wait_until='networkidle')
        page.locator('#username').fill(username)
        page.locator('#password').fill(password)
        page.locator('#loginForm button[type=submit]').click()
        page.locator('#workspace').wait_for(state='visible')
        page.wait_for_function("typeof latest !== 'undefined' && latest.mode === 'OFFLINE'")
        assert page.locator('#tab-overview').is_visible()
        assert page.locator('.nav-item').first.get_attribute('data-tab') == 'overview'
        assert page.locator('#pageTitle').inner_text() == 'Hệ thống'
        assert page.locator('#connectSystem').is_disabled()
        assert page.locator('#rosExecutorError').is_hidden()
        assert page.locator('#startNavigation').is_disabled()
        assert page.locator('#startMission').is_disabled()
        assert page.locator('#sendSingleGoal').is_disabled()
        assert page.locator('#globalStop').is_enabled()
        assert page.locator('#tab-delivery').is_hidden()
        assert page.evaluate('document.documentElement.scrollWidth <= innerWidth + 2')
        page.screenshot(path=str(output/'web_desktop_offline.png'), full_page=True)
        tabs = ['delivery', 'mapping', 'library', 'localization', 'devices', 'logs', 'overview']
        for tab in tabs:
            page.locator(f'.nav-item[data-tab="{tab}"]').click()
            page.locator(f'#tab-{tab}').wait_for(state='visible')
            assert page.evaluate('document.documentElement.scrollWidth <= innerWidth + 2'), tab
        page.set_viewport_size({'width': 390, 'height': 844})
        page.locator('.nav-item[data-tab="overview"]').click()
        page.screenshot(path=str(output/'web_mobile_offline.png'), full_page=True)
        assert page.evaluate('document.documentElement.scrollWidth <= innerWidth + 2'), 'mobile system overflow'
        assert page.locator('#startNavigation').is_disabled()
        assert page.locator('#globalStop').is_enabled()
        page.locator('#logoutMobile').click()
        page.locator('#loginView').wait_for(state='visible')
        assert not errors, '\n'.join(errors)
        browser.close()
    return {'checks': ['system first/default', 'offline controls locked', 'all existing tabs accessible',
                       'desktop/mobile width', 'global stop available',
                       'logout', 'no JavaScript exceptions'], 'result': 'PASS'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', required=True)
    parser.add_argument('--username', required=True)
    parser.add_argument('--password-file', required=True)
    parser.add_argument('--output', default='ui-artifacts')
    args = parser.parse_args()
    print(json.dumps(run(args.url, args.username, Path(args.password_file).read_text().strip(),
                         args.output), ensure_ascii=False, indent=2))
