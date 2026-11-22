// Three different robot arms (an SO-101, an i2rt YAM and a Franka Panda)
// tossing a ball down the line, catching it, and playing with visitors.
//
// Pure simulation logic: no DOM, no three.js, so it runs (and is tested) in
// Node as well as the browser. Every arm is driven by MuJoCo physics through
// its position actuators; the joint targets come from a damped-least-squares
// IK solve on a separate "ghost" MjData, so the controller is kinematically
// planned and the simulated arm tracks it with real dynamics and lag.
//
// All three arms run the same logic. What differs per robot (size, joints,
// gripper, which way its tool points, how it likes to reach low) lives in
// ARMS below. All of them work in one vertical plane, y = PLANE, which is
// also the plane a visitor drags the ball in.
//
// The ball is a mocap body: while held it rides the holder's grasp site, in
// flight it follows a ballistic arc.

const G = 9.81
const PLANE = 0.4 // every arm works in the vertical plane y = PLANE
const SEG = 1.6 // seconds per throw
const RELEASE = 0.3 // wind-up duration before release
const THROWS = [[0, 1], [1, 2], [2, 0]] // the long one sails over the middle arm
const CYCLE = SEG * THROWS.length

// Per-robot setup. Positions are world coordinates; every arm faces +y.
//   base     [x, y] of the arm's base (must match tools/build-models.mjs)
//   joints   arm joints the IK drives (not the gripper)
//   site     the point the IK steers, at the fingertips
//   tool     the tool's pointing axis in that site's frame
//   grip     gripper actuator and its open / holding-the-ball / resting commands
//   seed     a comfortable starting posture; the IK keeps drifting back toward it
//   z        how high the arm works (idle, release, catch)
//   tilt     how much the gripper prefers to point down (−) or up (+) from the shared aim
//   low      [z0, z1]: below z1 the gripper starts turning down, below z0 it points
//            straight at the table (measured: where each arm can still reach)
//   reach    half-width of the arm's zone around its base, for loose balls
//   top      highest point the arm reaches for a loose ball
//   side     how far toward its partner the arm leans to throw or catch
//   speed    how briskly it chases a new goal (rad/s of a critically damped spring)
//   kp       servo stiffness multiplier (the stock YAM servos are too soft to catch a fast throw)
//   kv       servo damping override (the stock SO-101 servos ring on fast moves)
const ARMS = [
  {
    name: 'so101', prefix: 'a0_', base: [-0.45, 0.07],
    joints: ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'],
    site: 'gripperframe', tool: [1, 0, 0],
    grip: { act: 'gripper', open: 0.9, hold: 0.3, rest: 0.55 },
    seed: [0, -0.6, 0.9, 0.6, 0],
    z: { idle: 0.18, release: 0.22, catch: 0.22 }, tilt: 0, low: [0.08, 0.15],
    reach: 0.2, top: 0.4, side: 0.04, speed: 13, kv: 8,
  },
  {
    name: 'yam', prefix: 'a1_', base: [0, 0],
    joints: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
    site: 'grasp_site', tool: [0, 0, 1],
    grip: { act: 'gripper', open: 0.037, hold: 0.018, rest: 0.022 },
    seed: [0, 1.0, 1.1, 0, 0, 0],
    z: { idle: 0.24, release: 0.3, catch: 0.32 }, tilt: 0, low: [0.12, 0.25],
    reach: 0.28, top: 0.58, side: 0.06, speed: 14, kp: 1.6,
  },
  {
    name: 'panda', prefix: 'a2_', base: [0.58, -0.15],
    joints: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'],
    site: 'grasp_site', tool: [0, 0, 1],
    grip: { act: 'actuator8', open: 255, hold: 127, rest: 190 },
    seed: [0, 0.3, 0, -2.2, 0, 2.6, 0.785],
    z: { idle: 0.46, release: 0.52, catch: 0.52 }, tilt: -0.8, low: [0.25, 0.4],
    reach: 0.36, top: 0.75, side: 0.08, speed: 12,
  },
]
export const BASES = ARMS.map(a => a.base)
const X = ARMS.map(a => a.base[0])
const SIZE = ARMS.map(a => (PLANE - a.base[1]) / 0.4) // motions scale with the arm

const v3 = {
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  scale: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  lerp: (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t],
  len: a => Math.hypot(a[0], a[1], a[2]),
  norm: a => {
    const l = Math.hypot(a[0], a[1], a[2]) || 1
    return [a[0] / l, a[1] / l, a[2] / l]
  },
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
}
const dist = (a, b) => v3.len(v3.sub(a, b))

const smooth = t => (t <= 0 ? 0 : t >= 1 ? 1 : t * t * t * (t * (6 * t - 15) + 10)) // min-jerk
const clamp01 = t => Math.max(0, Math.min(1, t))

// Exact step of a critically damped spring pulling e (with rate ev) to zero.
function damp(e, ev, w, dt) {
  const k = Math.exp(-w * dt)
  const c = [0, 1, 2].map(j => ev[j] + w * e[j])
  return [[0, 1, 2].map(j => (e[j] + c[j] * dt) * k), [0, 1, 2].map(j => (ev[j] - w * c[j] * dt) * k)]
}
// How long that spring needs to close a gap of D metres to within 2.5 cm.
function settleTime(D, w) {
  const r = 0.025 / Math.max(D, 0.026)
  let x = 1
  for (let k = 0; k < 30; k++) x += ((1 + x) * Math.exp(-x) - r) / (x * Math.exp(-x)) // Newton on (1+x)e^-x = r
  return x / w
}

// Solve A x = b for a small dense n×n system (Gaussian elimination, partial pivoting).
function solve(A, b, n) {
  const M = A.slice()
  const x = b.slice()
  for (let c = 0; c < n; c++) {
    let p = c
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r * n + c]) > Math.abs(M[p * n + c])) p = r
    if (p !== c) {
      for (let k = 0; k < n; k++) [M[c * n + k], M[p * n + k]] = [M[p * n + k], M[c * n + k]]
      ;[x[c], x[p]] = [x[p], x[c]]
    }
    const d = M[c * n + c] || 1e-12
    for (let r = c + 1; r < n; r++) {
      const f = M[r * n + c] / d
      if (!f) continue
      for (let k = c; k < n; k++) M[r * n + k] -= f * M[c * n + k]
      x[r] -= f * x[c]
    }
  }
  for (let r = n - 1; r >= 0; r--) {
    let s = x[r]
    for (let k = r + 1; k < n; k++) s -= M[r * n + k] * x[k]
    x[r] = s / (M[r * n + r] || 1e-12)
  }
  return x
}

// Where each arm likes to be, in world coordinates (arms face +y).
const towards = (i, j) => Math.sign(X[j] - X[i])
const idlePoint = i => [X[i], PLANE, ARMS[i].z.idle]
const releasePoint = (i, j) => [X[i] + ARMS[i].side * towards(i, j), PLANE, ARMS[i].z.release]
const catchPoint = (i, j) => [X[j] - 1.3 * ARMS[j].side * towards(i, j), PLANE, ARMS[j].z.catch]
// Tools point forward (+y), tilted toward the action.
const aim = (side, up) => v3.norm([side, 1, up])
const aimFor = (i, side, up) => aim(side, up + ARMS[i].tilt)
// The same, but turning toward the table when working low (each arm has its own band).
const aimAt = (i, side, up, z) => {
  const [z0, z1] = ARMS[i].low
  const u = smooth((z - z0) / (z1 - z0))
  return aim(side * u, -3.3 + (up + ARMS[i].tilt + 3.3) * u)
}
const DOWN = aim(0, -3.3)
const idleDir = i => aimFor(i, 0, 0.25)
const meetDir = (i, j) => aimFor(j, -0.5 * towards(i, j), 0.3) // jaws turned toward the thrower
// Longer throws fly longer (and higher).
const flightTime = (i, j) => 0.33 + 0.3 * Math.abs(catchPoint(i, j)[0] - releasePoint(i, j)[0])

function launchVelocity(p0, p1, T) {
  // p1 = p0 + v T - ½ g T² ẑ
  const d = v3.sub(p1, p0)
  return [d[0] / T, d[1] / T, d[2] / T + 0.5 * G * T]
}

export function createArms(mujoco, model, data) {
  const obj = mujoco.mjtObj
  const id = (type, name) => {
    const k = mujoco.mj_name2id(model, type.value, name)
    if (k < 0) throw new Error(`model has no ${name}`)
    return k
  }
  const nv = model.nv

  const arms = ARMS.map(cfg => {
    const joints = cfg.joints.map(n => id(obj.mjOBJ_JOINT, cfg.prefix + n))
    // the actuator driving each joint
    const act = joints.map(j => {
      for (let a = 0; a < model.nu; a++) if (model.actuator_trntype[a] === 0 && model.actuator_trnid[2 * a] === j) return a
      throw new Error(`no actuator for joint ${j}`)
    })
    if (cfg.kp) for (const a of act) { model.actuator_gainprm[10 * a] *= cfg.kp; model.actuator_biasprm[10 * a + 1] *= cfg.kp }
    if (cfg.kv) for (const a of act) model.actuator_biasprm[10 * a + 2] = -cfg.kv
    return {
      cfg,
      n: joints.length,
      site: id(obj.mjOBJ_SITE, cfg.prefix + cfg.site),
      qadr: joints.map(j => model.jnt_qposadr[j]),
      dadr: joints.map(j => model.jnt_dofadr[j]),
      range: joints.map(j => [model.jnt_range[2 * j], model.jnt_range[2 * j + 1]]),
      act,
      grip: id(obj.mjOBJ_ACTUATOR, cfg.prefix + cfg.grip.act),
      home: cfg.seed.slice(),
      q: cfg.seed.slice(), // IK solution, fed to the actuators
      // the commanded tool pose and its rate, kept continuous through every change of plan
      cmd: { p: [0, 0, 0], v: [0, 0, 0], d: [0, 1, 0], dv: [0, 0, 0] },
      // while in the routine: how far the command still is from the script, decaying to zero
      off: { e: [0, 0, 0], ev: [0, 0, 0], de: [0, 0, 0], dev: [0, 0, 0] },
    }
  })
  const ballBody = id(obj.mjOBJ_BODY, 'ball')
  const ballMocap = model.body_mocapid[ballBody]

  const ghost = new mujoco.MjData(model)
  const jacp = new mujoco.DoubleBuffer(3 * nv)
  const jacr = new mujoco.DoubleBuffer(3 * nv)

  // A visitor's pointer hovering over the scene: idle arms turn to look at it.
  const look = { target: null, p: null, w: 0 }

  // Turn the tool from `dir` toward the pointer, but only a little: a glance never
  // swings it more than ~25° off its usual direction, and fades out when the pointer
  // is right at the gripper (where the direction to it is meaningless).
  const GLANCE = 0.45 // rad
  function glance(dir, pos, idle) {
    const to = v3.sub(look.p, pos), d = v3.len(to)
    const w = 0.55 * look.w * idle * smooth((d - 0.05) / 0.1)
    if (w <= 0) return dir
    const g = v3.norm(v3.lerp(dir, v3.scale(to, 1 / d), w))
    const c = v3.dot(g, dir)
    if (c >= Math.cos(GLANCE)) return g
    const perp = v3.norm(v3.sub(g, v3.scale(dir, c)))
    return v3.add(v3.scale(dir, Math.cos(GLANCE)), v3.scale(perp, Math.sin(GLANCE)))
  }

  // Plan for one arm at cycle time t: where the grasp site should be, where the
  // gripper should point, and how open it is.
  // `wall` is continuous time, used for the idle sway so it never jumps when
  // the routine loops; `t` is the routine clock (mod CYCLE).
  function plan(i, t, wall = t) {
    const g = ARMS[i].grip
    const k0 = SIZE[i]
    const sway = [0.015 * k0 * Math.sin(wall * 1.3 + i), 0.01 * k0 * Math.sin(wall * 0.9 + 2 * i), 0.012 * k0 * Math.sin(wall * 1.7 + i)]
    let pos = v3.add(idlePoint(i), sway)
    let dir = idleDir(i)
    let busy = Infinity // how far (s) this arm is from its nearest throw or catch
    let grip = g.rest

    // Walk the throws of the previous, current and next loop, so phases that
    // straddle the loop boundary (like the first throw's wind-up) still apply.
    for (let k = -THROWS.length; k < 2 * THROWS.length; k++) {
      const [a, b] = THROWS[(k + THROWS.length) % THROWS.length]
      const s = k * SEG
      const tr = s + RELEASE
      const tc = tr + flightTime(a, b)
      const pr = releasePoint(a, b)
      const pc = catchPoint(a, b)
      const v0 = launchVelocity(pr, pc, flightTime(a, b))
      const vin = [v0[0], v0[1], v0[2] - G * flightTime(a, b)]
      const pw = v3.sub(pr, v3.scale(v0, 0.06 * SIZE[a])) // wind-up: back along the throw
      pw[2] = Math.max(pw[2], ARMS[a].z.idle * 0.8)
      const throwDir = aimFor(a, 0.45 * towards(a, b), 0.45)
      const meet = meetDir(a, b)

      if (i === a) busy = Math.min(busy, Math.max(0, s - 0.55 - t, t - (tr + 0.6)))
      if (i === b) busy = Math.min(busy, Math.max(0, tc - 0.5 - t, t - (tc + 0.6)))
      if (i === a) {
        if (t >= s - 0.55 && t < s) {
          // carry the ball from where it was caught to the wind-up point
          const u = smooth((t - (s - 0.55)) / 0.55)
          pos = v3.lerp(pos, pw, u)
          dir = v3.lerp(dir, throwDir, u)
          grip = g.hold
        } else if (t >= s && t < tr) {
          const u = clamp01((t - s) / RELEASE)
          pos = v3.lerp(pw, pr, u * u) // accelerate into the release
          dir = throwDir
          grip = g.hold
        } else if (t >= tr && t < tr + 0.6) {
          // follow through along the throw, easing out, then drift back to rest
          const u = (t - tr) / 0.6
          const motion = v3.norm(v3.sub(pr, pw))
          const over = v3.add(pr, v3.scale(motion, 0.07 * k0 * Math.sin(Math.min(1, u * 2.2) * Math.PI / 2)))
          pos = v3.lerp(over, pos, smooth(u))
          dir = v3.lerp(throwDir, dir, smooth(u))
          grip = g.open
        }
      }
      if (i === b) {
        if (t >= tc - 0.5 && t < tc) {
          const u = smooth((t - (tc - 0.5)) / 0.35)
          pos = v3.lerp(pos, pc, u)
          dir = v3.lerp(dir, meet, u)
          grip = g.open
        } else if (t >= tc && t < tc + 0.25) {
          // cushion the catch along the ball's velocity
          const u = (t - tc) / 0.25
          pos = v3.add(pc, v3.scale(v3.norm(vin), 0.05 * k0 * Math.sin(Math.PI * u * 0.5)))
          dir = meet
          grip = g.hold
        } else if (t >= tc + 0.25 && t < tc + 0.6) {
          const u = smooth((t - tc - 0.25) / 0.35)
          const cushioned = v3.add(pc, v3.scale(v3.norm(vin), 0.05 * k0))
          pos = v3.lerp(cushioned, pos, u)
          dir = v3.lerp(meet, dir, u)
          grip = g.hold
        }
      }
    }
    // only an idle arm glances at the pointer; it looks back well before its next throw or catch
    if (look.p && look.w > 0.001) dir = glance(v3.norm(dir), pos, smooth(busy / 0.3))
    return { pos, dir: v3.norm(dir), grip }
  }

  // Which arm holds the ball at cycle time t, or the flight it is on.
  function ballState(t) {
    for (let k = -1; k < THROWS.length; k++) {
      const [a, b] = THROWS[(k + THROWS.length) % THROWS.length]
      const s = k * SEG
      const tr = s + RELEASE
      const tc = tr + flightTime(a, b)
      if (t >= tr && t < tc) return { flight: { from: a, to: b, tr, tc } }
      const nextS = (k + 1) * SEG + RELEASE
      if (t >= tc && t < nextS) return { holder: b }
    }
    return { holder: THROWS[0][0] }
  }

  // The tool axis of arm `arm` in world coordinates, from an MjData's site frame.
  function toolAxis(d, arm) {
    const s = arm.site, xm = d.site_xmat, [a, b, c] = arm.cfg.tool
    return [0, 1, 2].map(r => xm[9 * s + 3 * r] * a + xm[9 * s + 3 * r + 1] * b + xm[9 * s + 3 * r + 2] * c)
  }

  function ikStep(targets, iters = 3) {
    for (let it = 0; it < iters; it++) {
      for (const arm of arms) arm.qadr.forEach((a, k) => (ghost.qpos[a] = arm.q[k]))
      mujoco.mj_kinematics(model, ghost)
      mujoco.mj_comPos(model, ghost)
      arms.forEach((arm, i) => {
        const tgt = targets[i]
        const n = arm.n
        mujoco.mj_jacSite(model, ghost, jacp, jacr, arm.site)
        const Jp = jacp.GetView()
        const Jr = jacr.GetView()
        const s = arm.site
        const p = [ghost.site_xpos[3 * s], ghost.site_xpos[3 * s + 1], ghost.site_xpos[3 * s + 2]]
        const z = toolAxis(ghost, arm)
        const ep = v3.sub(tgt.pos, p)
        const er = v3.scale(v3.cross(z, tgt.dir), 0.35) // weight orientation lower than position

        // 6×n Jacobian: position rows, then rotation rows projected off the tool axis (roll is free).
        const J = new Array(6 * n)
        for (let c = 0; c < n; c++) {
          const d = arm.dadr[c]
          const w = [Jr[d], Jr[nv + d], Jr[2 * nv + d]]
          const wp = v3.sub(w, v3.scale(z, v3.dot(w, z)))
          J[0 * n + c] = Jp[d]
          J[1 * n + c] = Jp[nv + d]
          J[2 * n + c] = Jp[2 * nv + d]
          J[3 * n + c] = wp[0] * 0.35
          J[4 * n + c] = wp[1] * 0.35
          J[5 * n + c] = wp[2] * 0.35
        }
        const e = [...ep, ...er]
        const lam2 = 0.002
        const A = new Array(36).fill(0)
        for (let r = 0; r < 6; r++)
          for (let c = 0; c < 6; c++) {
            let sum = r === c ? lam2 : 0
            for (let k = 0; k < n; k++) sum += J[r * n + k] * J[c * n + k]
            A[r * 6 + c] = sum
          }
        const y = solve(A, e, 6)
        for (let c = 0; c < n; c++) {
          let dq = 0
          for (let r = 0; r < 6; r++) dq += J[r * n + c] * y[r]
          dq += 0.02 * (arm.home[c] - arm.q[c]) // gentle pull toward a pleasant posture
          const [lo, hi] = arm.range[c]
          arm.q[c] = Math.min(hi - 0.02, Math.max(lo + 0.02, arm.q[c] + Math.max(-0.25, Math.min(0.25, dq))))
        }
      })
    }
  }

  // ---------------------------------------------------------------------------
  // Runtime. Three modes:
  //   routine — the scripted toss-and-catch loop above;
  //   hand    — a visitor is holding the ball (dragging it around): arms nearby
  //             reach for it, and one can take it if it is held right at its jaws;
  //   free    — the ball is loose: thrown by a visitor, or dropped. The arm that
  //             can get under its arc first commits to a catch point and goes
  //             there; the others watch. A ball on the table is picked up:
  //             above it, down, close the gripper, lift.
  // Every arm's commanded pose is continuous in position and velocity through
  // all of these changes of plan, so nothing ever snaps.
  // ---------------------------------------------------------------------------

  const BALL_R = 0.02
  const CATCH_DIST = 0.045
  const LAG = 0.07 // seconds the real arm trails its command
  const CATCH_BOOST = 1.6 // how much harder an arm pulls when going for a catch

  let mode = 'routine'
  let routineT = 0 // routine clock, seconds (mod CYCLE)
  let time = 0
  let lastTargets = null
  let holder = THROWS[0][0]
  let releasePos = null
  let releaseVel = null
  let flightCheck = null // routine flight we must verify was caught
  const ball = { p: [0, PLANE, 0.3], v: [0, 0, 0] }
  let held = { off: [0, 0, 0], t: -1 } // a just-caught ball settles into the jaws
  let catchPlan = null // { i, p, v, t }: the arm committed to a catch, where and when
  let bounces = 0
  let pick = null // { i, phase: 'above' | 'down' | 'close', t0, start }
  let restT = 0
  let snatch = { i: -1, t: 0 }

  const sitePos = s => [data.site_xpos[3 * s], data.site_xpos[3 * s + 1], data.site_xpos[3 * s + 2]]
  const gripPos = i => sitePos(arms[i].site)
  // Where a held ball sits: on the grasp site, plus whatever is left of the catch offset.
  const heldPos = i => v3.add(gripPos(i), v3.scale(held.off, Math.exp(-(time - held.t) / 0.05)))
  function setBall(p) {
    ball.p = p.slice()
    data.mocap_pos[3 * ballMocap] = p[0]
    data.mocap_pos[3 * ballMocap + 1] = p[1]
    data.mocap_pos[3 * ballMocap + 2] = p[2]
  }
  // Arm i now has the ball, wherever it was: it slides into the jaws over ~0.1 s.
  function attach(i) {
    held = { off: v3.sub(ball.p, gripPos(i)), t: time }
    holder = i
  }

  // The script's pose for arm i at the current routine clock, and its rate.
  function scripted(i, t = routineT, wall = time) {
    const a = plan(i, t, wall), b = plan(i, (t + 0.01) % CYCLE, wall + 0.01)
    return { ...a, vel: v3.scale(v3.sub(b.pos, a.pos), 100), dvel: v3.scale(v3.sub(b.dir, a.dir), 100) }
  }

  // Routine clock position at which arm `a` is about to wind up for its throw.
  function routineFor(a) {
    const k = THROWS.findIndex(([from]) => from === a)
    return ((k * SEG - 0.62) % CYCLE + CYCLE) % CYCLE
  }

  function enterRoutine(a) {
    mode = 'routine'
    routineT = routineFor(a)
    releasePos = null
    flightCheck = null
    catchPlan = null
    pick = null
    restT = 0
    snatch = { i: -1, t: 0 }
    // keep every arm exactly where it is, moving as it moves; the difference to
    // the script fades out smoothly
    arms.forEach((arm, i) => {
      const s = scripted(i)
      arm.off.e = v3.sub(arm.cmd.p, s.pos)
      arm.off.ev = v3.sub(arm.cmd.v, s.vel)
      arm.off.de = v3.sub(arm.cmd.d, s.dir)
      arm.off.dev = v3.sub(arm.cmd.dv, s.dvel)
    })
  }

  function enterFree(p, v) {
    mode = 'free'
    ball.p = p.slice()
    ball.v = v.slice()
    catchPlan = null
    pick = null
    restT = 0
    bounces = 0
    flightCheck = null
    releasePos = null
  }

  // A ball that is gone for good (rolled off, or nobody could reach it) drops
  // back in from above the nearest arm, which catches it.
  function dropIn() {
    let best = 0
    X.forEach((x, i) => { if (Math.abs(x - ball.p[0]) < Math.abs(X[best] - ball.p[0])) best = i })
    enterFree([X[best], PLANE, 2.0], [0, 0, 0])
    setBall(ball.p)
  }

  // Ballistic ball with floor bounces (the floor is the page: z = 0).
  function integrateBall(h) {
    const b = ball
    b.v[2] -= G * h
    for (let k = 0; k < 3; k++) b.p[k] += b.v[k] * h
    if (b.p[2] < BALL_R) {
      b.p[2] = BALL_R
      if (b.v[2] < 0) {
        b.v[2] = -b.v[2] * 0.55
        b.v[0] *= 0.8 // each bounce scrubs off some speed
        b.v[1] *= 0.8
        bounces++
      }
      if (Math.abs(b.v[2]) < 0.15) b.v[2] = 0
      const roll = Math.exp(-4 * h) // rolling friction on the table
      b.v[0] *= roll
      b.v[1] *= roll
    }
  }

  const inZone = (i, p) => {
    const c = ARMS[i]
    return Math.abs(p[0] - X[i]) < c.reach && Math.abs(p[1] - PLANE) < 0.12 && p[2] > 0.06 && p[2] < c.top
  }

  // Where along the ball's predicted arc can arm i be first, and when (from now)?
  // If it cannot make it anywhere, the point it comes closest to making, as an
  // attempt: a lunge that may or may not get there.
  function intercept(i) {
    const from = arms[i].cmd.p
    let p = ball.p.slice(), v = ball.v.slice()
    let best = null
    for (let t = 0.02; t < 1.8; t += 0.02) {
      v = [v[0], v[1], v[2] - G * 0.02]
      p = v3.add(p, v3.scale(v, 0.02))
      if (p[2] < 0.04) break
      if (!inZone(i, p)) continue
      const slack = t - settleTime(dist(p, from), ARMS[i].speed * CATCH_BOOST) - LAG
      if (slack >= 0) return { t, p, v, slack }
      if (!best || slack > best.slack) best = { t, p, v, slack }
    }
    return best && best.slack > -0.3 ? best : null
  }

  const watching = i => {
    const s = arms[i].cmd.p
    const look = v3.norm(v3.sub(ball.p, s))
    return v3.norm(v3.add(aimFor(i, 0, 0.2), v3.scale(look, 0.7)))
  }
  const restPose = (i, t) => {
    const k0 = SIZE[i]
    const sway = [0.01 * k0 * Math.sin(t * 1.3 + i), 0, 0.01 * k0 * Math.sin(t * 1.7 + i)]
    return { pos: v3.add(idlePoint(i), sway), dir: watching(i), grip: ARMS[i].grip.rest }
  }

  function freePlan(t, h) {
    const out = arms.map((_, i) => restPose(i, t))

    if (mode === 'hand') {
      // Arms near the ball lean in for it, jaws opening; the closer, the keener.
      arms.forEach((arm, i) => {
        const c = ARMS[i]
        const home = idlePoint(i)
        const dx = (ball.p[0] - X[i]) / (c.reach * 1.25)
        const dz = (ball.p[2] - home[2]) / (c.top * 0.9)
        const want = smooth(1 - Math.hypot(dx, dz))
        if (want <= 0) return
        const to = [ball.p[0], PLANE, Math.max(0.04, ball.p[2])]
        out[i] = {
          pos: v3.lerp(out[i].pos, to, 0.85 * want),
          dir: aimAt(i, 0.4 * Math.sign(ball.p[0] - X[i]), 0.25, to[2]),
          grip: c.grip.rest + (c.grip.open - c.grip.rest) * Math.min(1, 2 * want),
        }
      })
      return out
    }

    const airborne = ball.p[2] > BALL_R + 0.005 || ball.v[2] > 0.05
    if (airborne) {
      pick = null
      // Keep a committed catch unless the ball bounced off the plan or the moment passed.
      if (catchPlan && (catchPlan.bounces !== bounces || t > catchPlan.t + 0.15)) catchPlan = null
      if (!catchPlan) {
        // the earliest sure catch; failing that, the most promising attempt
        let best = null
        arms.forEach((_, i) => {
          const pl = intercept(i)
          if (!pl) return
          const better = !best || (pl.slack >= 0 && (best.slack < 0 || pl.t < best.t)) || (pl.slack < 0 && best.slack < 0 && pl.slack > best.slack)
          if (better) best = { ...pl, i }
        })
        if (best) catchPlan = { i: best.i, p: best.p, v: best.v, t: t + best.t, bounces }
      }
      if (catchPlan) {
        const i = catchPlan.i
        const side = -0.5 * Math.sign(catchPlan.v[0])
        // In the last moments, reach for where the ball really will be, not just the plan.
        const left = catchPlan.t - t
        let to = catchPlan.p
        if (left < 0.22) {
          const L = LAG
          const pb = [ball.p[0] + ball.v[0] * L, ball.p[1] + ball.v[1] * L, ball.p[2] + ball.v[2] * L - 0.5 * G * L * L]
          if (inZone(i, pb)) to = v3.lerp(catchPlan.p, pb, smooth((0.22 - left) / 0.22))
        }
        // Lead the command by how far the real gripper still trails it, so soft servos arrive on time.
        let lead = v3.scale(v3.sub(to, gripPos(i)), 0.6)
        const ll = v3.len(lead)
        if (ll > 0.06) lead = v3.scale(lead, 0.06 / ll)
        out[i] = { pos: v3.add(to, lead), dir: aimAt(i, side, 0.3, to[2]), grip: ARMS[i].grip.open }
      }
      return out
    }

    // On the table: the nearest arm that can reach it picks it up.
    catchPlan = null
    if (!pick) {
      let best = -1
      arms.forEach((_, i) => {
        const dx = Math.abs(ball.p[0] - X[i])
        if (dx < ARMS[i].reach && (best < 0 || dx < Math.abs(ball.p[0] - X[best]))) best = i
      })
      if (best >= 0) pick = { i: best, phase: 'above', t0: t, start: t }
    }
    if (pick) {
      const i = pick.i, s = gripPos(i), slow = v3.len(ball.v) < 0.2
      const above = v3.add(ball.p, [0, 0, 0.09])
      if (pick.phase === 'above' && slow && dist(s, above) < 0.025) Object.assign(pick, { phase: 'down', t0: t })
      if (pick.phase === 'down' && dist(s, ball.p) < 0.012) Object.assign(pick, { phase: 'close', t0: t })
      const g = ARMS[i].grip
      if (pick.phase === 'above') out[i] = { pos: above, dir: DOWN, grip: g.open }
      else out[i] = { pos: ball.p.slice(), dir: DOWN, grip: pick.phase === 'close' ? g.hold : g.open }
    }
    return out
  }

  // Bring each arm's commanded pose to this frame's plan, without any jumps.
  function command(goals, T) {
    arms.forEach((arm, i) => {
      const g = goals[i], c = arm.cmd
      if (mode === 'routine') {
        const o = arm.off
        ;[o.e, o.ev] = damp(o.e, o.ev, 9, T)
        ;[o.de, o.dev] = damp(o.de, o.dev, 9, T)
        c.p = v3.add(g.pos, o.e)
        c.v = v3.add(g.vel, o.ev)
        c.d = v3.norm(v3.add(g.dir, o.de))
        c.dv = v3.add(g.dvel, o.dev)
      } else {
        // a spring toward the goal: a new goal changes where it pulls, never where it is
        // (an arm going for a catch pulls harder)
        const w = ARMS[i].speed * (catchPlan && catchPlan.i === i ? CATCH_BOOST : 1)
        const [e, ev] = damp(v3.sub(c.p, g.pos), c.v, w, T)
        c.p = v3.add(g.pos, e)
        c.v = ev
        const [de, dev] = damp(v3.sub(c.d, g.dir), c.dv, ARMS[i].speed, T) // the wrist turns at its usual pace
        c.d = v3.norm(v3.add(g.dir, de))
        c.dv = dev
      }
    })
    return arms.map((arm, i) => ({ pos: arm.cmd.p, dir: arm.cmd.d, grip: goals[i].grip }))
  }

  function reset() {
    time = 0
    routineT = 0
    mode = 'routine'
    holder = THROWS[0][0]
    look.target = look.p = null
    look.w = 0
    // Solve each arm into its resting pose from its seed posture, and make that
    // the posture it drifts back toward.
    const rest = arms.map((_, i) => ({ pos: idlePoint(i), dir: idleDir(i), grip: ARMS[i].grip.rest }))
    for (let k = 0; k < 80; k++) ikStep(rest, 1)
    arms.forEach(arm => { arm.home = arm.q.slice() })
    const targets = arms.map((_, i) => scripted(i, 0, 0))
    arms.forEach((arm, i) => {
      arm.cmd = { p: targets[i].pos, v: targets[i].vel, d: targets[i].dir, dv: targets[i].dvel }
      arm.off = { e: [0, 0, 0], ev: [0, 0, 0], de: [0, 0, 0], dev: [0, 0, 0] }
    })
    for (let k = 0; k < 40; k++) ikStep(targets, 1)
    arms.forEach(arm => arm.qadr.forEach((a, k) => (data.qpos[a] = arm.q[k])))
    for (let i = 0; i < data.qvel.length; i++) data.qvel[i] = 0
    mujoco.mj_forward(model, data)
    lastTargets = targets
    held = { off: [0, 0, 0], t: -1 }
    setBall(gripPos(holder))
  }

  // Advance by dt seconds of simulated time.
  function step(dt) {
    const h = model.opt.timestep
    const n = Math.max(1, Math.round(dt / h))
    const T = n * h

    // follow the pointer's gaze target smoothly
    const lw = 1 - Math.exp(-T / 0.25)
    look.w += ((look.target ? 1 : 0) - look.w) * lw
    if (look.target) look.p = look.p ? v3.lerp(look.p, look.target, 1 - Math.exp(-T / 0.12)) : look.target.slice()

    const goals = mode === 'routine' ? arms.map((_, i) => scripted(i, (routineT + T) % CYCLE, time + T)) : freePlan(time, T)
    const targets = command(goals, T)
    ikStep(targets, 2)
    lastTargets = targets
    const ballHeld = mode === 'routine' && !releasePos
    arms.forEach((arm, i) => {
      arm.act.forEach((a, k) => (data.ctrl[a] = arm.q[k]))
      // whoever has the ball keeps a grip on it
      data.ctrl[arm.grip] = ballHeld && i === holder ? ARMS[i].grip.hold : targets[i].grip
    })

    for (let k = 0; k < n; k++) {
      time += h
      if (mode === 'routine') {
        routineT = (routineT + h) % CYCLE
        const st = ballState(routineT)
        if (st.flight) {
          if (!releasePos) {
            releasePos = ball.p.slice()
            releaseVel = launchVelocity(releasePos, catchPoint(st.flight.from, st.flight.to), st.flight.tc - st.flight.tr)
            flightCheck = st.flight
          }
          const u = routineT - st.flight.tr
          setBall([releasePos[0] + releaseVel[0] * u, releasePos[1] + releaseVel[1] * u, releasePos[2] + releaseVel[2] * u - 0.5 * G * u * u])
          ball.v = [releaseVel[0], releaseVel[1], releaseVel[2] - G * u]
        } else {
          if (flightCheck) {
            // The catcher should be there; if it was knocked off course, let the ball fly on.
            if (dist(gripPos(flightCheck.to), ball.p) > 0.07) { enterFree(ball.p, ball.v); continue }
            attach(flightCheck.to)
            flightCheck = null
          }
          releasePos = null
          holder = st.holder
          setBall(heldPos(holder))
        }
      } else if (mode === 'hand') {
        // held right at an arm's jaws for a moment, the arm takes it
        let near = -1
        arms.forEach((_, i) => { if (dist(gripPos(i), ball.p) < 0.035) near = i })
        snatch = near >= 0 && near === snatch.i ? { i: near, t: snatch.t + h } : { i: near, t: 0 }
        if (near >= 0 && snatch.t > 0.3) { attach(near); enterRoutine(near) }
      } else if (mode === 'free') {
        if (pick?.phase === 'close') {
          if (time - pick.t0 > 0.25) { attach(pick.i); enterRoutine(pick.i) }
        } else {
          integrateBall(h)
          setBall(ball.p)
          if (ball.p[2] > BALL_R + 0.01) {
            for (let i = 0; i < arms.length; i++) {
              if (dist(gripPos(i), ball.p) < CATCH_DIST) { attach(i); enterRoutine(i); break }
            }
          }
        }
        if (mode === 'free') {
          const resting = ball.p[2] <= BALL_R + 0.002 && v3.len(ball.v) < 0.12
          restT = resting ? restT + h : 0
          const lost = ball.p[0] < -1.0 || ball.p[0] > 1.3 || Math.abs(ball.p[1] - PLANE) > 0.6
          if (lost || (restT > 0.8 && !pick) || (pick && time - pick.start > 4)) dropIn()
        }
      }
      mujoco.mj_step(model, data)
    }
  }

  // --- visitor interaction (positions in MuJoCo world coordinates) ---
  function grab(p) {
    mode = 'hand'
    catchPlan = null
    pick = null
    releasePos = null
    flightCheck = null
    snatch = { i: -1, t: 0 }
    move(p)
  }
  function move(p) {
    if (mode !== 'hand') return
    const q = [Math.max(-1.0, Math.min(1.3, p[0])), p[1], Math.max(BALL_R, Math.min(1.0, p[2]))]
    setBall(q)
  }
  function release(v) {
    if (mode !== 'hand') return
    const sp = v3.len(v)
    const cap = 5.5
    enterFree(ball.p, sp > cap ? v3.scale(v, cap / sp) : v)
  }
  // The pointer is hovering at world point p (or has left: null).
  function lookAt(p) {
    look.target = p ? p.slice() : null
  }

  function dispose() {
    jacp.delete()
    jacr.delete()
    ghost.delete()
  }

  return {
    ikStep, reset, step, dispose, arms, ballBody, grab, move, release, lookAt,
    get mode() { return mode },
    get holding() { return mode === 'routine' && !releasePos },
    get flying() { return mode === 'free' || (mode === 'routine' && !!releasePos) }, get ball() { return ball }, get time() { return time }, get lastTargets() { return lastTargets },
    get pick() { return pick }, get catchPlan() { return catchPlan },
  }
}
