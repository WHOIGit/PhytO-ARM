// Renders docs/under-the-hood/index.html to MP4, one file per arm configuration.
// Usage: node render.cjs [outDir] [configKey|all] [fps] [scale]
// Needs Playwright (Chromium) and ffmpeg on PATH.
const { chromium } = require('playwright');
const { spawn } = require('child_process');
const fs = require('fs');
const path = require('path');

const [, , outDir = 'out', only = 'all', fpsArg = '30', scaleArg = '4'] = process.argv;
const fps = Number(fpsArg), scale = Number(scaleArg);

(async () => {
  fs.mkdirSync(outDir, { recursive: true });
  const browser = await chromium.launch();
  const page = await browser.newPage();
  await page.addInitScript(() => { window.__RENDER__ = true; });
  await page.goto('file://' + path.resolve(__dirname, 'index.html'));
  await page.waitForFunction(() => window.PA);
  const { W, H, DUR, keys } = await page.evaluate(() => ({ W: PA.W, H: PA.H, DUR: PA.DUR, keys: PA.keys }));

  for (const k of keys) {
    if (only !== 'all' && only !== k) continue;
    const wav = path.join(outDir, `phyto-arm-${k}.wav`);
    const mp4 = path.join(outDir, `phyto-arm-${k}.mp4`);
    fs.writeFileSync(wav, Buffer.from(await page.evaluate((key) => PA.audio(key), k), 'base64'));
    await page.evaluate((key) => PA.use(key), k);

    const ff = spawn('ffmpeg', ['-y', '-loglevel', 'error',
      '-f', 'rawvideo', '-pix_fmt', 'rgba', '-s', `${W}x${H}`, '-r', String(fps), '-i', '-',
      '-i', wav, '-vf', `scale=${W * scale}:${H * scale}:flags=neighbor`,
      '-c:v', 'libx264', '-preset', 'medium', '-tune', 'animation', '-crf', '16', '-pix_fmt', 'yuv420p',
      '-c:a', 'aac', '-b:a', '192k', '-shortest', '-movflags', '+faststart', mp4],
      { stdio: ['pipe', 'inherit', 'inherit'] });

    const total = Math.floor(DUR * fps);
    for (let i = 0; i < total; i += 60) {
      const n = Math.min(60, total - i);
      const b64 = await page.evaluate(([t, n, f]) => PA.frames(t, n, f), [i / fps, n, fps]);
      if (!ff.stdin.write(Buffer.from(b64, 'base64'))) await new Promise((r) => ff.stdin.once('drain', r));
    }
    ff.stdin.end();
    await new Promise((r) => ff.on('close', r));
    fs.unlinkSync(wav);
    console.log('wrote', mp4);
  }
  await browser.close();
})();
