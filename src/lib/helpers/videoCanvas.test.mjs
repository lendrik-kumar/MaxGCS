// Run: node --test src/lib/helpers/videoCanvas.test.mjs   (Node 22.6+ strips the TypeScript types)
import test from 'node:test';
import assert from 'node:assert/strict';
import { canvasPlacement, canvasBoxFit, pictureInSurface, CANVAS_W, CANVAS_H } from './videoCanvas.ts';

// The same table as `src-tauri/src/video/canvas.rs` — the two implementations must agree.
test('small pictures are centred at their own size (never upscaled)', () => {
  assert.deepEqual(canvasPlacement(720, 480), { w: 720, h: 480, x: 600, y: 300 });
  assert.deepEqual(canvasPlacement(1280, 720), { w: 1280, h: 720, x: 320, y: 180 });
  assert.deepEqual(canvasPlacement(640, 480), { w: 640, h: 480, x: 640, y: 300 });
  assert.deepEqual(canvasPlacement(1920, 1080), { w: 1920, h: 1080, x: 0, y: 0 });
});

test('pictures pad on the free axis only', () => {
  assert.deepEqual(canvasPlacement(1440, 1080), { w: 1440, h: 1080, x: 240, y: 0 });
  assert.deepEqual(canvasPlacement(1920, 800), { w: 1920, h: 800, x: 0, y: 140 });
});

test('oversized pictures shrink to fit, aspect preserved', () => {
  assert.deepEqual(canvasPlacement(3840, 2160), { w: 1920, h: 1080, x: 0, y: 0 });
  assert.deepEqual(canvasPlacement(4096, 2160), { w: 1920, h: 1012, x: 0, y: 34 });
  assert.deepEqual(canvasPlacement(1080, 2160), { w: 540, h: 1080, x: 690, y: 0 });
});

test('placement stays on the canvas, never upscales, stays centred', () => {
  for (const [sw, sh] of [[1, 1], [2, 2], [321, 241], [719, 479], [1919, 1079], [1921, 1081], [7680, 4320], [100, 5000]]) {
    const p = canvasPlacement(sw, sh);
    assert.ok(p.w >= 1 && p.h >= 1, `${sw}x${sh} collapsed`);
    assert.ok(p.x + p.w <= CANVAS_W && p.y + p.h <= CANVAS_H, `${sw}x${sh} overflows`);
    if (sw <= CANVAS_W && sh <= CANVAS_H) assert.deepEqual([p.w, p.h], [sw, sh]);
    assert.ok(Math.abs(p.x * 2 + p.w - CANVAS_W) <= 3 && Math.abs(p.y * 2 + p.h - CANVAS_H) <= 3, `${sw}x${sh} off-centre`);
  }
});

test('degenerate input is not placed', () => {
  assert.deepEqual(canvasPlacement(0, 480), { w: 0, h: 0, x: 0, y: 0 });
  assert.equal(pictureInSurface(0, 0, 800, 450), null);
  assert.equal(pictureInSurface(720, 480, 0, 0), null);
});

test('the canvas fits any surface without distortion', () => {
  assert.deepEqual(canvasBoxFit(1920, 1080), { w: 1920, h: 1080 });
  const wide = canvasBoxFit(2000, 500); // height-bound → pillarboxed 16:9
  assert.ok(Math.abs(wide.w / wide.h - 16 / 9) < 1e-9 && wide.h === 500);
  const tall = canvasBoxFit(400, 2000); // width-bound → letterboxed 16:9
  assert.ok(Math.abs(tall.w / tall.h - 16 / 9) < 1e-9 && tall.w === 400);
});

test('the picture keeps its pixel ratio and centre in a surface of any shape', () => {
  for (const [bw, bh] of [[1920, 1080], [640, 360], [800, 600], [300, 150], [1000, 1000]]) {
    const r = pictureInSurface(720, 480, bw, bh);
    assert.ok(Math.abs(r.w / r.h - 720 / 480) < 1e-9, `aspect changed in ${bw}x${bh}`);
    assert.ok(Math.abs(r.left + r.w / 2 - bw / 2) < 1.5 && Math.abs(r.top + r.h / 2 - bh / 2) < 1.5, `off-centre in ${bw}x${bh}`);
  }
  // On a 1920x1080 surface it is exactly 1:1 with the source: no scaling at all.
  const one = pictureInSurface(720, 480, 1920, 1080);
  assert.deepEqual([one.w, one.h, one.left, one.top], [720, 480, 600, 300]);
  // On a half-size surface everything is exactly half (a uniform scale of the whole canvas).
  const half = pictureInSurface(720, 480, 960, 540);
  assert.deepEqual([half.w, half.h, half.left, half.top], [360, 240, 300, 150]);
});
