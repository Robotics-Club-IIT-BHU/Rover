#!/usr/bin/env python3
"""
captive_login.py. Log into a captive WiFi portal from the terminal.

Campus WiFi normally means: plug in a monitor, open a browser, type
credentials. On a headless rover that is a trip to the lab every time the
lease expires. This does it over SSH instead.

Three modes:
    detect    is a portal in the way?                (exit 0 online, 1 portal)
    discover  read the login form, write a template  (exit 0 written)
    login     submit stored credentials              (exit 0 online)

    ros2 run athena_remote captive_login --ros-args -p mode:=detect
    ros2 run athena_remote captive_login --ros-args -p mode:=discover
    ros2 run athena_remote captive_login

It is a ROS node only so that `ros2 run` can start it and `--ros-args -p`
can set the mode; it publishes and subscribes to nothing.

Credentials live in ~/.config/athena_remote/portal.yaml, written 0600 and
deliberately outside the repository so they cannot be committed.
"""

import os
import re
import stat
import sys
import time
import urllib.parse
import urllib.request
import urllib.error

import rclpy
import yaml
from rclpy.node import Node

CONFIG_PATH = os.path.expanduser('~/.config/athena_remote/portal.yaml')

# Plain-HTTP URLs with known-constant responses.  A portal has to intercept
# these to redirect a browser, which is exactly how it gives itself away.
PROBES = [
    'http://connectivitycheck.gstatic.com/generate_204',
    'http://detectportal.firefox.com/success.txt',
    'http://neverssl.com',
]

# The portal usually needs a moment to move the session from "captive" to
# "authorised" after the form is accepted, so retry before giving up.
LOGIN_RETRIES = 3
LOGIN_RETRY_WAIT = 2     # s

MAX_REDIRECT_HOPS = 5


def http_get(url, timeout=8):
    """GET without automatically following HTTP redirects. Returns (status, headers, body)."""
    class NoRedirect(urllib.request.HTTPRedirectHandler):
        def redirect_request(self, *a, **k):
            return None

    op = urllib.request.build_opener(NoRedirect)
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
    try:
        r = op.open(req, timeout=timeout)
        return r.status, dict(r.headers), r.read().decode('utf-8', 'ignore')
    except urllib.error.HTTPError as e:
        return e.code, dict(e.headers), e.read().decode('utf-8', 'ignore')
    except Exception as e:
        return None, {'error': str(e)}, ''


def extract_redirect(body, current_url):
    """Extract HTML meta-refresh or JavaScript location assignments."""
    if not body:
        return None

    # 1. HTML Meta-refresh
    m = re.search(r'(?i)<meta[^>]+http-equiv=["\']?refresh["\']?[^>]*content=["\']?[0-9]*\s*;\s*url=([^"\'\s>]+)', body)
    if not m:
        m = re.search(r'(?i)<meta[^>]+content=["\']?[0-9]*\s*;\s*url=([^"\'\s>]+)["\']?[^>]*http-equiv=["\']?refresh', body)
    if m:
        return urllib.parse.urljoin(current_url, m.group(1).strip('\'"'))

    # 2. JavaScript redirects: window.location, location.href, location.replace(...)
    m = re.search(r'''(?i)(?:window\.)?location(?:\.href|\.replace|\.assign)?\s*(?:=|\()\s*["']([^'"]+)["']''', body)
    if m:
        return urllib.parse.urljoin(current_url, m.group(1))

    return None


def internet_ok():
    st, hdr, _ = http_get(PROBES[0], timeout=6)
    return st == 204 and not hdr.get('Location') and not hdr.get('location')


def find_portal(max_hops=MAX_REDIRECT_HOPS):
    """Trace redirects to the actual portal landing page, or None if online."""
    for probe in PROBES:
        current = probe
        resolved = False
        for _ in range(max_hops):
            st, hdr, body = http_get(current)
            if st is None:
                break

            loc = hdr.get('Location') or hdr.get('location')
            if st in (301, 302, 303, 307, 308) and loc:
                current = urllib.parse.urljoin(current, loc)
                resolved = True
                continue

            redir = extract_redirect(body, current)
            if redir:
                current = redir
                resolved = True
                continue

            if resolved or (st == 200 and 'generate_204' in probe):
                return current
            break

    return None


def _attr(tag_attrs, name, default='', unquoted=r'[^\s>]+'):
    """Value of one HTML attribute, quoted or bare, or `default` if absent.

    A real HTML parser is not used on purpose: this has to run on a rover
    with no network, so it cannot depend on anything outside the standard
    library, and portal login forms are simple enough for a regex.

    `unquoted` exists because an empty bare attribute is meaningful for
    `value=` (a blank field) but not for the others, so only that caller
    accepts a zero-length match.
    """
    m = re.search(
        r'''(?i)\b%s\s*=\s*(?:["']([^"']*)["']|(%s))''' % (name, unquoted),
        tag_attrs)
    if not m:
        return default
    return m.group(1) if m.group(1) is not None else m.group(2)


def parse_forms(html, base_url):
    """Extract forms as list of (action, method, {field_name: {'value': v, 'type': t}})."""
    forms = []
    for fm in re.finditer(r'(?is)<form\b(.*?)>(.*?)</form>', html):
        attrs, inner = fm.group(1), fm.group(2)
        action = _attr(attrs, 'action')
        method = _attr(attrs, 'method', default='get').lower()

        fields = {}
        for inp in re.finditer(r'(?is)<input\b([^>]*)>', inner):
            a = inp.group(1)
            name_val = _attr(a, 'name', default=None)
            if name_val is None:
                continue
            fields[name_val] = {
                'value': _attr(a, 'value', unquoted=r'[^\s>]*'),
                'type': _attr(a, 'type', default='text').lower(),
            }

        forms.append((urllib.parse.urljoin(base_url, action), method, fields))
    return forms


class CaptiveLogin(Node):
    def __init__(self):
        super().__init__('captive_login')
        self.declare_parameter('mode', 'login')

    def detect(self):
        print('checking for a captive portal...')
        if internet_ok():
            print('  internet is reachable - no portal in the way.')
            return 0
        url = find_portal()
        if not url:
            print('  no internet, and no portal detected either.')
            print('  Check the WiFi association first:  nmcli dev wifi')
            return 2
        print(f'  PORTAL: {url}')
        print('  next:  mode:=discover')
        return 1

    def discover(self):
        url = find_portal()
        if not url:
            print('no portal found (already online?). Nothing to discover.')
            return 1
        print(f'portal: {url}\nfetching the login form...\n')
        st, hdr, body = http_get(url)
        if st is None:
            print(f'  could not fetch it: {hdr.get("error")}')
            return 2

        forms = parse_forms(body, url)
        if not forms:
            print('  no <form> found. Page preview:')
            print(f'\n--- first 800 bytes of the page ---\n{body[:800]}')
            return 2

        for i, (action, method, fields) in enumerate(forms, 1):
            print(f'  form {i}:  {method.upper()}  {action}')
            for name, meta in fields.items():
                kind = meta['type']
                shown = '' if kind == 'password' else meta['value']
                tag = {'password': '  <-- your password',
                       'text': '  <-- probably your username',
                       'hidden': '  (hidden - refreshed dynamically at login)'}.get(kind, '')
                print(f'      {name:22s} type={kind:9s} value={shown!r}{tag}')
            print()

        action, method, fields = forms[0]
        if not os.path.exists(CONFIG_PATH):
            self._write_template(action, method, fields)
        else:
            print(f'{CONFIG_PATH} already exists - not overwriting.')
        return 0

    def _write_template(self, action, method, fields):
        os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
        lines = ['# athena_remote captive portal login',
                 '# Generated by mode:=discover. Fill in your username and password.',
                 f'url: {action!r}', f'method: {method}', 'fields:']
        for name, meta in fields.items():
            v = '' if meta['type'] in ('text', 'password') else meta['value']
            lines.append(f'  {name}: {v!r}')
        lines += ["success_marker: ''", '']

        with open(CONFIG_PATH, 'w') as f:
            f.write('\n'.join(lines))
        os.chmod(CONFIG_PATH, stat.S_IRUSR | stat.S_IWUSR)
        print(f'wrote template to {CONFIG_PATH} (mode 0600).')
        print('Fill in your username and password, then run:')
        print('  ros2 run athena_remote captive_login')

    def login(self):
        if internet_ok():
            print('already online - nothing to do.')
            return 0
        if not os.path.exists(CONFIG_PATH):
            print(f'no config at {CONFIG_PATH}')
            print('run:  ros2 run athena_remote captive_login --ros-args -p mode:=discover')
            return 2

        with open(CONFIG_PATH) as f:
            cfg = yaml.safe_load(f) or {}

        target_url = cfg.get('url')
        method = (cfg.get('method') or 'post').lower()
        post_fields = {k: str(v) for k, v in (cfg.get('fields') or {}).items()}

        if not any(post_fields.values()):
            print('every field is blank - fill in credentials first.')
            return 2

        target_url, method = self._refresh_from_live_form(
            target_url, method, post_fields)

        if not target_url:
            print(f'{CONFIG_PATH} has no url and live portal discovery failed.')
            return 2

        body = self._submit(target_url, method, post_fields)
        if body is None:
            return 2

        marker = cfg.get('success_marker') or ''
        if marker and marker in body:
            print(f'  portal returned success marker ({marker!r})')

        # The portal's own reply is not proof of anything - some return 200
        # with a failure page - so the only test that counts is whether real
        # traffic gets out afterwards.
        for _ in range(LOGIN_RETRIES):
            time.sleep(LOGIN_RETRY_WAIT)
            if internet_ok():
                print('  ONLINE - internet is reachable.')
                return 0

        print('  still offline. Check credentials in ~/.config/athena_remote/portal.yaml')
        return 2

    @staticmethod
    def _refresh_from_live_form(target_url, method, post_fields):
        """Overlay the live form's hidden fields onto the stored credentials.

        FortiGate and other enterprise portals mint a fresh magic/CSRF token
        per session, so the hidden values captured by `discover` are stale by
        the time anyone logs in.  Fetching the form now and copying its
        hidden fields over is what makes a stored config keep working.

        Mutates post_fields; returns the (possibly updated) url and method.
        """
        portal_url = find_portal()
        if not portal_url:
            return target_url, method
        st, _, body = http_get(portal_url)
        live_forms = parse_forms(body, portal_url) if st == 200 else []
        if not live_forms:
            return target_url, method

        live_action, live_method, live_fields = live_forms[0]
        for fname, fmeta in live_fields.items():
            if fmeta['type'] == 'hidden' and fmeta['value']:
                post_fields[fname] = fmeta['value']
            elif fname not in post_fields:
                post_fields[fname] = fmeta['value']
        return live_action, live_method

    @staticmethod
    def _submit(target_url, method, post_fields):
        """POST or GET the login form. Returns the body, or None on failure."""
        data = urllib.parse.urlencode(post_fields).encode()
        print(f'submitting to {target_url} ({method.upper()}) ...')
        try:
            if method == 'get':
                req = urllib.request.Request(f'{target_url}?{data.decode()}')
            else:
                req = urllib.request.Request(target_url, data=data)
            req.add_header('User-Agent', 'Mozilla/5.0')
            with urllib.request.urlopen(req, timeout=15) as r:
                return r.read().decode('utf-8', 'ignore')
        except Exception as e:
            print(f'  request failed: {e}')
            return None

    def run(self):
        mode = str(self.get_parameter('mode').value).lower()
        if mode == 'detect':
            return self.detect()
        if mode == 'discover':
            return self.discover()
        return self.login()


def main(args=None):
    rclpy.init(args=args)
    node = CaptiveLogin()
    try:
        rc = node.run()
    except KeyboardInterrupt:
        rc = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
