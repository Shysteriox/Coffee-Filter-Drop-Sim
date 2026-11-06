// Coffee filter drop simulator: physics + 2D lattice-Boltzmann fluid.
// Plain script (no modules) so index.html works straight from file://.
(function () {
  'use strict';

  const RHO = 1.2;        // air density, kg/m^3
  const MU = 1.81e-5;     // air dynamic viscosity, Pa*s
  const G = 9.81;
  const FILTER_MASS = 0.95e-3; // kg per filter (lab: 0.9-1.0 g)

  // ---------- random numbers ----------
  function mulberry32(seed) {
    let a = seed >>> 0;
    return function () {
      a = (a + 0x6D2B79F5) >>> 0;
      let t = a;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }
  function randn(rng) {
    let u = 0;
    while (u === 0) u = rng();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * rng());
  }

  // ---------- drop physics ----------
  // P = { m, cd, body, bend, air, height }
  // body = { A, Aside, R, h, Lcp, Ip, Iy, flut }  (Ip, Iy are per kg)
  function terminalVelocity(P) {
    return Math.sqrt(2 * P.m * G / (RHO * P.cd * P.body.A * (1 - 0.12 * P.bend)));
  }

  function drawRelease(sig, rng) {
    const d2r = Math.PI / 180;
    return {
      tx: sig.tilt * d2r * randn(rng), tz: sig.tilt * d2r * randn(rng),
      wx: sig.spin * d2r * randn(rng), wz: sig.spin * d2r * randn(rng),
      wy: sig.yaw * d2r * randn(rng),
      asym: 0.002 * randn(rng),
      ph: 2 * Math.PI * rng(), phi: 2 * Math.PI * rng(),
    };
  }

  function createDrop(P, rel) {
    return {
      t: 0, x: 0, z: 0, y: P.height, vx: 0, vy: 0, vz: 0, ay: G,
      tx: rel.tx, tz: rel.tz, wx: rel.wx, wz: rel.wz,
      yaw: 0, wy: rel.wy, asym: rel.asym,
      gx: 0, gz: 0, nx: 0, nz: 0, vmax: 0, landed: false,
      sp: rel.ph || 0, sa: rel.phi || 0,   // vortex-shedding phase + the axis it rocks about
    };
  }

  // Semi-implicit Euler. vy is downward speed (positive while falling).
  function stepDrop(s, P, dt, rng) {
    const b = P.body, m = P.m;
    // room drafts: Ornstein-Uhlenbeck horizontal air velocity, 0.5 s memory
    const a = dt / 0.5, sa = Math.sqrt(2 * a);
    s.gx += -a * s.gx + P.air * sa * randn(rng);
    s.gz += -a * s.gz + P.air * sa * randn(rng);
    // wake / flutter forcing: unit OU noise, 0.15 s memory
    const c = dt / 0.15, sc = Math.sqrt(2 * c);
    s.nx += -c * s.nx + sc * randn(rng);
    s.nz += -c * s.nz + sc * randn(rng);

    // velocity of body relative to the air
    const rx = s.vx - s.gx, ry = -s.vy, rz = s.vz - s.gz;
    const U = Math.hypot(rx, ry, rz) + 1e-9;
    const q = 0.5 * RHO * U * U;
    const th = Math.hypot(s.tx, s.tz);

    // quadratic drag on the tilted frontal area
    const Aeff = (b.A * Math.cos(th) + b.Aside * Math.sin(th)) * (1 - 0.12 * P.bend);
    const D = q * P.cd * Aeff;
    let fx = -D * rx / U, fy = -D * ry / U, fz = -D * rz / U;
    // a tilted cone glides toward the side it leans to
    if (th > 1e-6) {
      const L = q * b.A * 0.35 * Math.sin(2 * th) * 0.5;
      fx += L * s.tx / th; fz += L * s.tz / th;
    }
    s.vx += fx / m * dt; s.vz += fz / m * dt;
    s.ay = G - fy / m;
    s.vy += s.ay * dt;
    s.x += s.vx * dt; s.z += s.vz * dt; s.y -= s.vy * dt;

    // tilt: aerodynamic restoring torque (centre of pressure behind CM) + damping + forcing
    const Ip = m * b.Ip;
    const K = q * b.A * b.Lcp * (1 - 0.7 * P.bend);
    const C = 0.07 * RHO * U * b.A * b.R * b.R;
    const kd = 0.5 * RHO * b.A * b.R * 1.5;
    const fl = q * b.A * b.R * (b.flut + 0.03 * P.bend);
    // vortex shedding: the wake sheds vortices alternately off each side (Strouhal ~0.2),
    // pushing the rim up on one side then the other. Light stacks fall slowly, so the
    // shedding rate lands near their natural rocking rate and they rock; heavy ones don't.
    s.sp += 2 * Math.PI * 0.2 * U / (2 * b.R) * dt;
    s.sa += 0.4 * Math.sqrt(dt) * randn(rng);
    const fs = 0.008 * q * b.A * b.R * Math.cos(s.sp);
    const tqx = -K * s.tx - C * s.wx + kd * Math.abs(s.gx) * s.gx + fl * s.nx + fs * Math.cos(s.sa);
    const tqz = -K * s.tz - C * s.wz + kd * Math.abs(s.gz) * s.gz + fl * s.nz + fs * Math.sin(s.sa);
    s.wx += tqx / Ip * dt; s.wz += tqz / Ip * dt;
    s.tx += s.wx * dt; s.tz += s.wz * dt;
    const th2 = Math.hypot(s.tx, s.tz);
    if (th2 > 1.45) { s.tx *= 1.45 / th2; s.tz *= 1.45 / th2; s.wx *= 0.5; s.wz *= 0.5; }

    // spin about the vertical axis: fluting asymmetry drives it, air damps it
    const Iy = m * b.Iy;
    const ty = q * b.A * b.R * s.asym - 0.1 * RHO * U * b.A * b.R * b.R * s.wy;
    s.wy += ty / Iy * dt; s.yaw += s.wy * dt;

    s.t += dt;
    if (s.vy > s.vmax) s.vmax = s.vy;
    if (s.y <= 0) { s.y = 0; s.landed = true; }
  }

  function summarize(s) {
    const r2d = 180 / Math.PI;
    return {
      tilt: Math.hypot(s.tx, s.tz) * r2d,
      yaw: Math.abs(s.yaw) * r2d,
      drift: Math.hypot(s.x, s.z) * 100,
      time: s.t,
      vmax: s.vmax,
    };
  }

  function runDrop(P, sig, rng, dt) {
    const rel = drawRelease(sig, rng);
    const s = createDrop(P, rel);
    while (!s.landed && s.t < 30) stepDrop(s, P, dt, rng);
    const out = summarize(s);
    out.release = Math.hypot(rel.tx, rel.tz) * 180 / Math.PI;
    return out;
  }

  function monteCarlo(P, sig, N, seed) {
    const rng = mulberry32(seed);
    const out = [];
    for (let i = 0; i < N; i++) out.push(runDrop(P, sig, rng, 0.002));
    return out;
  }

  function stats(arr) {
    const n = arr.length;
    const mean = arr.reduce((a, b) => a + b, 0) / n;
    const sd = Math.sqrt(arr.reduce((a, b) => a + (b - mean) * (b - mean), 0) / Math.max(1, n - 1));
    return { mean, sd };
  }

  // ---------- D2Q9 lattice Boltzmann (BGK, half-way bounce-back) ----------
  // Air enters at the bottom (y = 0) and leaves at the top: the frame of the falling object.
  const EX = [0, 1, 0, -1, 0, 1, -1, -1, 1];
  const EY = [0, 0, 1, 0, -1, 1, 1, -1, -1];
  const WT = [4 / 9, 1 / 9, 1 / 9, 1 / 9, 1 / 9, 1 / 36, 1 / 36, 1 / 36, 1 / 36];
  const OPP = [0, 3, 4, 1, 2, 7, 8, 5, 6];

  class LBM {
    constructor(W, H, tau) {
      this.W = W; this.H = H; this.N = W * H;
      this.omega = 1 / tau;
      this.f = new Float32Array(9 * this.N);
      this.g = new Float32Array(9 * this.N);
      this.ux = new Float32Array(this.N);
      this.uy = new Float32Array(this.N);
      this.solid = new Uint8Array(this.N);
      this.shade = new Uint8Array(this.N);
      this.reset(0.05);
    }
    setEq(buf, i, rho, ux, uy) {
      const N = this.N, u2 = 1.5 * (ux * ux + uy * uy);
      for (let k = 0; k < 9; k++) {
        const eu = EX[k] * ux + EY[k] * uy;
        buf[k * N + i] = WT[k] * rho * (1 + 3 * eu + 4.5 * eu * eu - u2);
      }
    }
    reset(uIn) {
      for (let i = 0; i < this.N; i++) {
        this.setEq(this.f, i, 1, 0, uIn); this.setEq(this.g, i, 1, 0, uIn);
        this.ux[i] = 0; this.uy[i] = uIn;
      }
    }
    // pts: body-frame points {x, y (m, about CM), s (0..1 height shade)}
    setObstacle(pts, angle, cx, cy, cell) {
      const W = this.W, H = this.H;
      const old = this.solid;
      const solid = new Uint8Array(this.N);
      const shade = this.shade;
      const c = Math.cos(angle), s = Math.sin(angle);
      for (const p of pts) {
        const x = p.x * c + p.y * s, y = -p.x * s + p.y * c;
        const gx = Math.round(cx + x / cell), gy = Math.round(cy + y / cell);
        if (gx < 0 || gx >= W || gy < 1 || gy >= H - 1) continue;
        const i = gy * W + gx;
        solid[i] = 1; shade[i] = p.s * 255;
      }
      // cells that were solid and are now fluid start at rest
      for (let i = 0; i < this.N; i++) {
        if (old[i] && !solid[i]) {
          this.setEq(this.f, i, 1, 0, 0); this.setEq(this.g, i, 1, 0, 0);
        }
      }
      this.solid = solid;
    }
    step(uIn) {
      const W = this.W, H = this.H, N = this.N, om = this.omega, sol = this.solid;
      let f = this.f, g = this.g;
      // tiny sideways jitter at the inlet (like real room air) so the wake can go
      // unstable and shed vortices left/right instead of staying perfectly mirrored
      for (let x = 0; x < W; x++) this.setEq(f, x, 1, uIn * 0.03 * (Math.random() - 0.5), uIn);
      const ux = this.ux, uy = this.uy;
      const fk = new Float32Array(9);
      for (let y = 0; y < H; y++) {
        for (let x = 0; x < W; x++) {
          const i = y * W + x;
          if (sol[i]) { ux[i] = 0; uy[i] = 0; continue; }
          let rho = 0;
          for (let k = 0; k < 9; k++) { fk[k] = f[k * N + i]; rho += fk[k]; }
          if (!(rho > 0.2 && rho < 5)) { rho = 1; for (let k = 0; k < 9; k++) fk[k] = WT[k]; }
          const vx = (fk[1] - fk[3] + fk[5] - fk[6] - fk[7] + fk[8]) / rho;
          const vy = (fk[2] - fk[4] + fk[5] + fk[6] - fk[7] - fk[8]) / rho;
          ux[i] = vx; uy[i] = vy;
          const u2 = 1.5 * (vx * vx + vy * vy);
          for (let k = 0; k < 9; k++) {
            const eu = EX[k] * vx + EY[k] * vy;
            const post = fk[k] + om * (WT[k] * rho * (1 + 3 * eu + 4.5 * eu * eu - u2) - fk[k]);
            const ys = y + EY[k];
            if (ys < 0 || ys >= H) continue;
            let xs = x + EX[k];
            if (xs < 0) xs += W; else if (xs >= W) xs -= W;
            const j = ys * W + xs;
            if (sol[j]) g[OPP[k] * N + i] = post;
            else g[k * N + j] = post;
          }
        }
      }
      // zero-gradient outflow at the top
      const top = (H - 1) * W, below = (H - 2) * W;
      for (let k = 0; k < 9; k++) {
        const o = k * N;
        for (let x = 0; x < W; x++) g[o + top + x] = g[o + below + x];
      }
      this.f = g; this.g = f;
    }
    // bilinear velocity sample at grid coords
    sample(x, y, out) {
      const W = this.W, H = this.H;
      if (x < 0) x += W; else if (x >= W) x -= W;
      const x0 = Math.floor(x), y0 = Math.max(0, Math.min(H - 2, Math.floor(y)));
      const x1 = (x0 + 1) % W, fx = x - x0, fy = Math.max(0, Math.min(1, y - y0));
      const i00 = y0 * W + x0, i10 = y0 * W + x1, i01 = i00 + W, i11 = i10 + W;
      const ux = this.ux, uy = this.uy;
      out[0] = (ux[i00] * (1 - fx) + ux[i10] * fx) * (1 - fy) + (ux[i01] * (1 - fx) + ux[i11] * fx) * fy;
      out[1] = (uy[i00] * (1 - fx) + uy[i10] * fx) * (1 - fy) + (uy[i01] * (1 - fx) + uy[i11] * fx) * fy;
      return out;
    }
  }

  // turbo colormap (polynomial fit), t in [0,1] -> [r,g,b] 0..255
  function turbo(t) {
    t = Math.max(0, Math.min(1, t));
    const r = 0.13572138 + t * (4.61539260 + t * (-42.66032258 + t * (132.13108234 + t * (-152.94239396 + t * 59.28637943))));
    const g = 0.09140261 + t * (2.19418839 + t * (4.84296658 + t * (-14.18503333 + t * (4.27729857 + t * 2.82956604))));
    const b = 0.10667330 + t * (12.64194608 + t * (-60.58204836 + t * (110.36276771 + t * (-89.90310912 + t * 27.34824973))));
    return [Math.max(0, Math.min(255, r * 255)), Math.max(0, Math.min(255, g * 255)), Math.max(0, Math.min(255, b * 255))];
  }

  window.Sim = {
    RHO, MU, G, FILTER_MASS,
    mulberry32, randn, terminalVelocity, drawRelease, createDrop, stepDrop,
    summarize, runDrop, monteCarlo, stats, LBM, turbo,
  };
})();
