// Three different robot arms (an SO-101, an i2rt YAM and a Franka Panda)
// tossing a ball down the line and catching it.
//
// Pure simulation logic: no DOM, no three.js, so it runs (and is tested) in
// Node as well as the browser. Every arm is driven by MuJoCo physics through
// its position actuators; the joint targets come from a damped-least-squares
// IK solve on a separate "ghost" MjData, so the controller is kinematically
// planned and the simulated arm tracks it with real dynamics and lag.
//
// All three arms run the same logic. What differs per robot (size, joints,
// gripper, which way its tool points) lives in ARMS below.
//
// The ball is a mocap body: while held it rides the holder's grasp site, in
// flight it follows the exact ballistic arc toward the catcher's planned
// catch point.

const G = 9.81
const SEG = 1.6 // seconds per throw
const RELEASE = 0.3 // wind-up duration before release
const THROWS = [[0, 1], [1, 2], [2, 0]] // the long one sails over the middle arm
const CYCLE = SEG * THROWS.length

// Per-robot setup. Positions are world coordinates; every arm faces +y.
//   base     x of the arm's base (must match scene.xml)
//   joints   arm joints the IK drives (not the gripper)
//   site     the point the IK steers, at the fingertips
//   tool     the tool's pointing axis in that site's frame
//   grip     gripper actuator and its open / holding-the-ball / resting commands
//   seed     a comfortable starting posture; the IK keeps drifting back toward it
//   y, z     how far forward and how high the arm works (idle, release, catch)
//   tilt     how much the gripper prefers to point down (−) or up (+) from the shared aim
//   reach    half-width of the catch zone around the base, for loose balls
//   side     how far toward its partner the arm leans to throw or catch
const ARMS = [
  {
    name: 'so101', prefix: 'a0_', base: -0.45,
    joints: ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'],
    site: 'gripperframe', tool: [1, 0, 0],
    grip: { act: 'gripper', open: 0.9, hold: 0.3, rest: 0.55 },
    seed: [0, -0.6, 0.9, 0.6, 0],
    y: 0.29, z: { idle: 0.18, release: 0.22, catch: 0.22 }, tilt: 0, reach: 0.2, side: 0.04,
    zone: { y: [0.18, 0.4], z: [0.08, 0.36] },
  },
  {
    name: 'yam', prefix: 'a1_', base: 0,
    joints: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
    site: 'grasp_site', tool: [0, 0, 1],
    grip: { act: 'gripper', open: 0.037, hold: 0.018, rest: 0.022 },
    seed: [0, 1.0, 1.1, 0, 0, 0],
    y: 0.39, z: { idle: 0.24, release: 0.3, catch: 0.32 }, tilt: 0, reach: 0.28, side: 0.06,
    zone: { y: [0.22, 0.58], z: [0.1, 0.55] },
  },
  {
    name: 'panda', prefix: 'a2_', base: 0.58,
    joints: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'],
    site: 'grasp_site', tool: [0, 0, 1],
    grip: { act: 'actuator8', open: 255, hold: 127, rest: 190 },
    seed: [0, 0.3, 0, -2.2, 0, 2.6, 0.785],
    y: 0.55, z: { idle: 0.46, release: 0.52, catch: 0.52 }, tilt: -0.8, reach: 0.36, side: 0.08,
    zone: { y: [0.4, 0.75], z: [0.3, 0.75] },
  },
]
const BASES = ARMS.map(a => a.base)

const v3 = {
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  scale: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  lerp: (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t],
  norm: a => {
    const l = Math.hypot(a[0], a[1], a[2]) || 1
    return [a[0] / l, a[1] / l, a[2] / l]
  },
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
}

const smooth = t => (t <= 0 ? 0 : t >= 1 ? 1 : t * t * t * (t * (6 * t - 15) + 10)) // min-jerk
const clamp01 = t => Math.max(0, Math.min(1, t))

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
const towards = (i, j) => Math.sign(BASES[j] - BASES[i])
const idlePoint = i => [BASES[i], ARMS[i].y, ARMS[i].z.idle]
const releasePoint = (i, j) => [BASES[i] + ARMS[i].side * towards(i, j), ARMS[i].y, ARMS[i].z.release]
const catchPoint = (i, j) => [BASES[j] - 1.3 * ARMS[j].side * towards(i, j), ARMS[j].y, ARMS[j].z.catch]
// Tools point forward (+y), tilted toward the action.
const aim = (side, up) => v3.norm([side, 1, up])
const aimFor = (i, side, up) => aim(side, up + ARMS[i].tilt)
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
    }
  })
  const ballBody = id(obj.mjOBJ_BODY, 'ball')
  const ballMocap = model.body_mocapid[ballBody]

  const ghost = new mujoco.MjData(model)
  const jacp = new mujoco.DoubleBuffer(3 * nv)
  const jacr = new mujoco.DoubleBuffer(3 * nv)

  // Plan for one arm at cycle time t: where the grasp site should be, where the
  // gripper should point, and how open it is.
  // `wall` is continuous time, used for the idle sway so it never jumps when
  // the routine loops; `t` is the routine clock (mod CYCLE).
  function plan(i, t, wall = t) {
    const g = ARMS[i].grip
    const k0 = ARMS[i].y / 0.39 // sway scaled to the arm's size
    const sway = [0.015 * k0 * Math.sin(wall * 1.3 + i), 0.01 * k0 * Math.sin(wall * 0.9 + 2 * i), 0.012 * k0 * Math.sin(wall * 1.7 + i)]
    let pos = v3.add(idlePoint(i), sway)
    let dir = idleDir(i)
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
      const pw = v3.sub(pr, v3.scale(v0, 0.06 * (ARMS[a].y / 0.39))) // wind-up: back along the throw
      pw[2] = Math.max(pw[2], ARMS[a].z.idle * 0.8)
      const throwDir = aimFor(a, 0.45 * towards(a, b), 0.45)
      const meet = meetDir(a, b)

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
  //   hand    — a visitor is holding the ball (dragging it around);
  //   free    — the ball is loose: thrown by a visitor, or dropped. Every frame
  //             the arms predict its arc, the best-placed arm goes to intercept
  //             it, the others watch it. A ball that lands is picked up or
  //             handed back, and the routine resumes from whoever has it.
  // ---------------------------------------------------------------------------

  const BALL_R = 0.02
  const CATCH_DIST = 0.045
  const ARM_SPEED = 1.6 // m/s the planner assumes a gripper can travel

  let mode = 'routine'
  let routineT = 0 // routine clock, seconds (mod CYCLE)
  let time = 0
  let lastTargets = null
  let holder = THROWS[0][0]
  let releasePos = null
  let releaseVel = null
  let flightCheck = null // routine flight we must verify was caught
  const ball = { p: [0, 0.4, 0.3], v: [0, 0, 0] }
  let catcher = -1
  let restT = 0
  let picker = -1
  let pickPhase = 0
  const blend = arms.map(() => ({ dp: [0, 0, 0], dd: [0, 0, 0] }))

  const sitePos = s => [data.site_xpos[3 * s], data.site_xpos[3 * s + 1], data.site_xpos[3 * s + 2]]
  // Where a held ball sits: on the grasp site.
  const gripPos = i => sitePos(arms[i].site)
  function setBall(p) {
    ball.p = p.slice()
    data.mocap_pos[3 * ballMocap] = p[0]
    data.mocap_pos[3 * ballMocap + 1] = p[1]
    data.mocap_pos[3 * ballMocap + 2] = p[2]
  }

  // Remember where each arm was headed, so a change of plan blends in
  // smoothly instead of snapping.
  function rebase(newTargets) {
    if (!lastTargets) return
    arms.forEach((_, i) => {
      blend[i].dp = v3.add(blend[i].dp, v3.sub(lastTargets[i].pos, newTargets[i].pos))
      blend[i].dd = v3.add(blend[i].dd, v3.sub(lastTargets[i].dir, newTargets[i].dir))
    })
  }

  // Routine clock position at which arm `a` is about to wind up for its throw.
  function routineFor(a) {
    const k = THROWS.findIndex(([from]) => from === a)
    return ((k * SEG - 0.62) % CYCLE + CYCLE) % CYCLE
  }

  function enterRoutine(a) {
    holder = a
    mode = 'routine'
    routineT = routineFor(a)
    releasePos = null
    flightCheck = null
    catcher = -1
    picker = -1
    restT = 0
    rebase(arms.map((_, i) => plan(i, routineT, time)))
  }

  function enterFree(p, v) {
    mode = 'free'
    ball.p = p.slice()
    ball.v = v.slice()
    catcher = -1
    picker = -1
    restT = 0
    flightCheck = null
    releasePos = null
  }

  // Ballistic ball with floor bounces (the floor is the page: z = 0).
  function integrateBall(h) {
    const b = ball
    b.v[2] -= G * h
    for (let k = 0; k < 3; k++) b.p[k] += b.v[k] * h
    if (b.p[2] < BALL_R) {
      b.p[2] = BALL_R
      if (b.v[2] < 0) b.v[2] = -b.v[2] * 0.55
      if (Math.abs(b.v[2]) < 0.15) b.v[2] = 0
      const roll = Math.exp(-1.8 * h) // rolling friction on the table
      b.v[0] *= roll
      b.v[1] *= roll
    }
  }

  const inZone = (i, p) => {
    const c = ARMS[i]
    return Math.abs(p[0] - BASES[i]) < c.reach && p[1] > c.zone.y[0] && p[1] < c.zone.y[1] && p[2] > c.zone.z[0] && p[2] < c.zone.z[1]
  }

  // Where along the ball's predicted arc can arm i be first, and when?
  function intercept(i) {
    const s = gripPos(i)
    let p = ball.p.slice(), v = ball.v.slice()
    for (let t = 0.02; t < 1.6; t += 0.02) {
      v = [v[0], v[1], v[2] - G * 0.02]
      p = v3.add(p, v3.scale(v, 0.02))
      if (p[2] < 0.03) return null
      if (!inZone(i, p)) continue
      const travel = Math.hypot(p[0] - s[0], p[1] - s[1], p[2] - s[2]) / ARM_SPEED + 0.08
      if (travel <= t) return { t, p, v }
    }
    return null
  }

  const watching = (i, extra = 0) => {
    const s = sitePos(arms[i].site)
    const look = v3.norm(v3.sub(ball.p, s))
    return v3.norm(v3.add(aimFor(i, 0, 0.2), v3.scale(look, 0.7 + extra)))
  }

  function freePlan(t) {
    const out = arms.map((_, i) => {
      const k0 = ARMS[i].y / 0.39
      const sway = [0.01 * k0 * Math.sin(t * 1.3 + i), 0, 0.01 * k0 * Math.sin(t * 1.7 + i)]
      return { pos: v3.add(idlePoint(i), sway), dir: watching(i), grip: ARMS[i].grip.rest }
    })
    if (mode === 'hand') return out

    // In the air: choose the arm that gets there first (keep the current one if it still can).
    const airborne = ball.p[2] > BALL_R + 0.005 || ball.v[2] > 0.05
    if (airborne) {
      const plans = arms.map((_, i) => intercept(i))
      if (catcher < 0 || !plans[catcher]) {
        let best = -1
        plans.forEach((pl, i) => { if (pl && (best < 0 || pl.t < plans[best].t)) best = i })
        catcher = best
      }
      if (catcher >= 0) {
        const pl = plans[catcher]
        const side = Math.sign(-pl.v[0]) || 0
        out[catcher] = { pos: pl.p, dir: aimFor(catcher, 0.5 * side, 0.25), grip: ARMS[catcher].grip.open }
      }
      return out
    }

    // On the floor: the nearest arm that can reach it picks it up.
    catcher = -1
    if (picker < 0) {
      let best = -1, bd = 1e9
      arms.forEach((_, i) => {
        const c = ARMS[i]
        const dx = Math.abs(ball.p[0] - BASES[i])
        if (dx < c.reach && ball.p[1] > c.zone.y[0] && ball.p[1] < c.zone.y[1] && dx < bd) { bd = dx; best = i }
      })
      picker = best
      pickPhase = 0
    }
    if (picker >= 0) {
      const down = aim(0, -0.9)
      const above = v3.add(ball.p, [0, 0, 0.1])
      const s = gripPos(picker)
      if (pickPhase === 0 && Math.hypot(...v3.sub(above, s)) < 0.03) pickPhase = 1
      out[picker] = { pos: pickPhase ? v3.add(ball.p, [0, 0, 0.005]) : above, dir: down, grip: ARMS[picker].grip.open }
    }
    return out
  }

  function decayBlend(targets, dt) {
    const k = Math.exp(-dt / 0.22)
    return targets.map((tg, i) => {
      const b = blend[i]
      b.dp = v3.scale(b.dp, k)
      b.dd = v3.scale(b.dd, k)
      return { pos: v3.add(tg.pos, b.dp), dir: v3.norm(v3.add(tg.dir, b.dd)), grip: tg.grip }
    })
  }

  function reset() {
    time = 0
    routineT = 0
    mode = 'routine'
    // Solve each arm into its resting pose from its seed posture, and make that
    // the posture it drifts back toward.
    const rest = arms.map((_, i) => ({ pos: idlePoint(i), dir: idleDir(i), grip: ARMS[i].grip.rest }))
    for (let k = 0; k < 80; k++) ikStep(rest, 1)
    arms.forEach(arm => { arm.home = arm.q.slice() })
    const targets = arms.map((_, i) => plan(i, 0))
    for (let k = 0; k < 40; k++) ikStep(targets, 1)
    arms.forEach(arm => arm.qadr.forEach((a, k) => (data.qpos[a] = arm.q[k])))
    for (let i = 0; i < data.qvel.length; i++) data.qvel[i] = 0
    mujoco.mj_forward(model, data)
    lastTargets = targets
    setBall(gripPos(holder))
  }

  // Advance by dt seconds of simulated time.
  function step(dt) {
    const h = model.opt.timestep
    const n = Math.max(1, Math.round(dt / h))
    let targets = mode === 'routine' ? arms.map((_, i) => plan(i, (routineT + n * h) % CYCLE, time + n * h)) : freePlan(time)
    const goal = targets
    targets = decayBlend(targets, n * h)
    ikStep(targets, 2)
    lastTargets = goal
    arms.forEach((arm, i) => {
      arm.act.forEach((a, k) => (data.ctrl[a] = arm.q[k]))
      data.ctrl[arm.grip] = targets[i].grip
    })

    for (let k = 0; k < n; k++) {
      time += h
      if (mode === 'routine') {
        routineT = (routineT + h) % CYCLE
        const st = ballState(routineT)
        if (st.flight) {
          if (!releasePos) {
            releasePos = gripPos(st.flight.from)
            releaseVel = launchVelocity(releasePos, catchPoint(st.flight.from, st.flight.to), st.flight.tc - st.flight.tr)
            flightCheck = st.flight
          }
          const u = routineT - st.flight.tr
          setBall([releasePos[0] + releaseVel[0] * u, releasePos[1] + releaseVel[1] * u, releasePos[2] + releaseVel[2] * u - 0.5 * G * u * u])
          ball.v = [releaseVel[0], releaseVel[1], releaseVel[2] - G * u]
        } else {
          if (flightCheck) {
            // The catcher should be there; if it was knocked off course, let the ball fly on.
            const s = gripPos(flightCheck.to)
            if (Math.hypot(...v3.sub(s, ball.p)) > 0.07) { enterFree(ball.p, ball.v); continue }
            flightCheck = null
          }
          releasePos = null
          holder = st.holder
          setBall(gripPos(holder))
        }
      } else if (mode === 'free') {
        integrateBall(h)
        setBall(ball.p)
        for (let i = 0; i < arms.length; i++) {
          if (Math.hypot(...v3.sub(gripPos(i), ball.p)) < CATCH_DIST) { enterRoutine(i); break }
        }
        if (mode === 'free') {
          const resting = ball.p[2] <= BALL_R + 0.002 && Math.hypot(...ball.v) < 0.12
          restT = resting ? restT + h : 0
          const lost = Math.abs(ball.p[0]) > 1.5 || ball.p[1] < -0.8 || ball.p[1] > 1.4
          if (lost || (restT > 1.2 && picker < 0) || restT > 4) {
            // Hand it back to the nearest arm.
            let best = 0
            BASES.forEach((x, i) => { if (Math.abs(x - ball.p[0]) < Math.abs(BASES[best] - ball.p[0])) best = i })
            enterRoutine(best)
            setBall(gripPos(best))
          }
        }
      }
      mujoco.mj_step(model, data)
    }
  }

  // --- visitor interaction (positions in MuJoCo world coordinates) ---
  function grab(p) {
    mode = 'hand'
    catcher = -1
    picker = -1
    releasePos = null
    flightCheck = null
    rebase(freePlan(time))
    move(p)
  }
  function move(p) {
    if (mode !== 'hand') return
    const q = [Math.max(-1.2, Math.min(1.2, p[0])), p[1], Math.max(BALL_R, Math.min(1.0, p[2]))]
    setBall(q)
  }
  function release(v) {
    if (mode !== 'hand') return
    const sp = Math.hypot(...v)
    const cap = 5.5
    enterFree(ball.p, sp > cap ? v3.scale(v, cap / sp) : v)
  }

  function dispose() {
    jacp.delete()
    jacr.delete()
    ghost.delete()
  }

  return {
    ikStep, reset, step, dispose, arms, ballBody, grab, move, release,
    get mode() { return mode },
    get flying() { return mode === 'free' || (mode === 'routine' && !!releasePos) }, get ball() { return ball }, get time() { return time }, get lastTargets() { return lastTargets },
  }
}
