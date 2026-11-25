// Record the README turntable of the detailed CAD assembly in the three.js viewer (cad/viewer/).
//   node scripts/record_turntable.mjs          -> out/turntable/f0000.png ... (one PNG per frame)
// then (25 fps; revolution 1 solid, revolution 2 X-ray):
//   ffmpeg -framerate 25 -i out/turntable/f%04d.png -vf scale=960:-2 -c:v libx264 -pix_fmt yuv420p -crf 24 docs/media/turntable.mp4
//   ffmpeg -t 8 -i docs/media/turntable.mp4 -vf "fps=10,scale=600:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=64:stats_mode=diff[p];[b][p]paletteuse=dither=none:diff_mode=rectangle" docs/media/turntable.gif
//   cp out/turntable/f0290.png docs/media/xray.png        (GIF = first revolution only, to stay under 5 MB)
// The page's requestAnimationFrame is replaced by a manual tick and the viewer's own auto-rotate is sped up
// so one tick = one frame's angle: the output is frame-exact and independent of render speed.
// Headless Chromium + SwiftShader WebGL. Exits 1 on any console error or a failed load.
// This is the CAD model (kestrel/cad_detail.py, exported by `python -m kestrel.export_viewer`); the aircraft is not built yet.
import { createRequire } from 'node:module';
import { mkdirSync, readFileSync, rmSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
const require = createRequire(import.meta.url);
const { chromium } = require('playwright');

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const viewer = path.join(root, 'cad', 'viewer');
const out = path.join(root, 'out', 'turntable');
rmSync(out, { recursive: true, force: true });
mkdirSync(out, { recursive: true });

const audit = JSON.parse(readFileSync(path.join(viewer, 'audit.json'), 'utf8'));
if (!audit.ok || audit.conflicts !== 0) { console.error('interference audit is not clean:', audit); process.exit(1); }
const FRAMES_PER_REV = 200;            // 8 s per revolution at 25 fps
// OrbitControls r147 turns 2*pi/3600 * autoRotateSpeed per update, so this speed gives one frame's angle per update.
const SPEED = 3600 / FRAMES_PER_REV;
const SECTIONS = [
  { caption: `Kestrel quadplane VTOL · detailed CAD assembly, ${audit.parts} parts, ${audit.conflicts} clashes`, xray: false, dims: false },
  { caption: 'X-ray foam: cross spar, outer spars, servos, torque rods, pod electronics', xray: true, dims: false },
];

const b = await chromium.launch({ args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const ctx = await b.newContext({ viewport: { width: 1500, height: 900 }, colorScheme: 'dark', deviceScaleFactor: 1 });
const page = await ctx.newPage();
const errors = [];
page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
page.on('pageerror', (e) => errors.push(String(e)));
// Serve cad/viewer at http://kestrel.local/ with the data under viewer/ (the layout the page expects).
await page.route('http://kestrel.local/**', (route) => {
  const p = new URL(route.request().url()).pathname;
  const file = p === '/' ? path.join(viewer, 'index.html') : path.join(viewer, p.replace(/^\/viewer\//, ''));
  route.fulfill({ body: readFileSync(file), contentType: file.endsWith('.html') ? 'text/html' : 'application/json' });
});
await page.addInitScript(() => {
  const q = [];
  window.requestAnimationFrame = (cb) => { q.push(cb); return q.length; };
  window.__tick = () => { const cbs = q.splice(0); cbs.forEach((cb) => cb(performance.now())); };
  // Keep a handle on the viewer's OrbitControls so each tick can turn a whole frame's worth of angle.
  let three;
  Object.defineProperty(window, 'THREE', { configurable: true, get: () => three, set: (v) => {
    three = v; let OC;
    Object.defineProperty(v, 'OrbitControls', { configurable: true, get: () => OC, set: (C) => {
      OC = class extends C { constructor(...a) { super(...a); window.__controls = this; } };
    } });
  } });
});
await page.goto('http://kestrel.local/');
await page.waitForFunction(() => document.getElementById('loading').hidden, null, { timeout: 180000 });
await page.addStyleTag({ content: '.views, .stage > div:not(#labels):not(#cap) { display: none !important } .stage { border: 0; border-radius: 0 }' +
  ' #cap { position: absolute; left: 0; right: 0; bottom: 0; padding: 10px 16px; font: 600 17px system-ui, sans-serif; color: #e6edf3;' +
  ' background: rgba(13,17,23,0.82); display: flex; justify-content: space-between } #cap span:last-child { color: #8b949e; font-weight: 500 }' });
await page.evaluate(() => {
  const d = document.createElement('div'); d.id = 'cap';
  d.innerHTML = '<span id="cap-t"></span><span>CAD model · not yet built</span>';
  document.querySelector('.stage').appendChild(d);
});
await page.click('#dims');                       // dimensions on by default; the turntable reads better without them
await page.click('#spin');
await page.evaluate((v) => { window.__controls.autoRotateSpeed = v; window.__controls.enableDamping = false;
  window.__controls.object.position.multiplyScalar(1.22); window.__controls.update(); }, SPEED);   // room for the wingtips at every angle
const stage = page.locator('.stage');
let f = 0;
for (const s of SECTIONS) {
  const xrayOn = await page.getAttribute('#xray', 'aria-pressed') === 'true';
  if (xrayOn !== s.xray) await page.click('#xray');
  await page.evaluate((t) => { document.getElementById('cap-t').textContent = t; }, s.caption);
  for (let i = 0; i < FRAMES_PER_REV; i++) {
    await page.evaluate(() => window.__tick());
    await stage.screenshot({ path: path.join(out, `f${String(f++).padStart(4, '0')}.png`) });
  }
}
await ctx.close();
await b.close();
if (errors.length) { console.error(errors); process.exit(1); }
console.log(`wrote ${f} frames to ${out}`);
