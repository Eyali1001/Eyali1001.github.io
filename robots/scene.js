// Mounts the MuJoCo arm scene into <figure id="arms">.
//
// Contract with the page: this module is optional decoration. Every failure
// (no WebGL, no WASM, slow network, a thrown error mid-frame, a lost GPU
// context) ends in the same place: the figure is removed and the page is
// exactly what it would have been without JavaScript.

import * as THREE from '../vendor/three/three.module.min.js'
import loadMujoco from '../vendor/mujoco/mujoco.js'
import { createArms, BASES } from './arms.js?v=79e89a0143'

const BASE = new URL('./models/', import.meta.url)
const LOAD_TIMEOUT_MS = 30000


export async function mount(figure) {
  const canvas = figure.querySelector('canvas')
  let renderer, sim, model, data, raf, observer, resizer
  let disposed = false

  const fail = err => {
    if (disposed) return
    disposed = true
    if (err && typeof console !== 'undefined') console.info('[arms] skipped:', err.message || err)
    cancelAnimationFrame(raf)
    observer?.disconnect()
    resizer?.disconnect()
    try { renderer?.dispose() } catch {}
    try { sim?.dispose(); data?.delete(); model?.delete() } catch {}
    figure.remove()
    document.documentElement.classList.remove('has-arms')
  }

  try {
    const timeout = new Promise((_, rej) => setTimeout(() => rej(new Error('timeout')), LOAD_TIMEOUT_MS))
    const fetchBytes = async name => {
      const r = await fetch(new URL(name, BASE))
      if (!r.ok) throw new Error(`${name}: ${r.status}`)
      return new Uint8Array(await r.arrayBuffer())
    }

    const [mujoco, files] = await Promise.race([
      Promise.all([
        loadMujoco(),
        // every file the scene needs, listed by tools/build-models.mjs
        fetch(new URL('manifest.json', BASE), { cache: 'no-cache' }).then(r => (r.ok ? r.json() : Promise.reject(new Error('manifest'))))
          .then(({ v, files }) => Promise.all(files.map(async f => [f, await fetchBytes(`${f}?v=${v}`)]))),
      ]),
      timeout,
    ])
    if (disposed) return

    const made = new Set()
    const mkdirs = dir => { if (!dir || made.has(dir)) return; mkdirs(dir.slice(0, dir.lastIndexOf('/'))); try { mujoco.FS.mkdir(dir) } catch {} made.add(dir) }
    for (const [name, bytes] of files) {
      mkdirs(`/arms/${name}`.slice(0, `/arms/${name}`.lastIndexOf('/')))
      mujoco.FS.writeFile(`/arms/${name}`, bytes)
    }
    model = mujoco.MjModel.from_xml_path('/arms/scene.xml')
    data = new mujoco.MjData(model)
    sim = createArms(mujoco, model, data)
    sim.reset()

    renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true, powerPreference: 'low-power' })
    renderer.setClearColor(0xffffff, 0)
    renderer.outputColorSpace = THREE.SRGBColorSpace
    canvas.addEventListener('webglcontextlost', e => { e.preventDefault(); fail(new Error('context lost')) })

    const scene = new THREE.Scene()
    scene.add(new THREE.HemisphereLight(0xffffff, 0xd8d8d8, 2.2))
    const key = new THREE.DirectionalLight(0xffffff, 1.6)
    key.position.set(1.5, 3, 2.5)
    scene.add(key)

    // MuJoCo is z-up; three.js is y-up.
    const world = new THREE.Group()
    world.rotation.x = -Math.PI / 2
    scene.add(world)

    // No floor, no table: just a faint contact shadow under each base, so the
    // arms stand on the page rather than float above it.
    const shadowTex = (() => {
      const c = document.createElement('canvas')
      c.width = c.height = 64
      const g = c.getContext('2d')
      const grd = g.createRadialGradient(32, 32, 0, 32, 32, 32)
      grd.addColorStop(0, 'rgba(0,0,0,0.22)')
      grd.addColorStop(1, 'rgba(0,0,0,0)')
      g.fillStyle = grd
      g.fillRect(0, 0, 64, 64)
      return new THREE.CanvasTexture(c)
    })()
    for (const [x, y] of BASES) {
      const disc = new THREE.Mesh(new THREE.PlaneGeometry(0.26, 0.26), new THREE.MeshBasicMaterial({ map: shadowTex, transparent: true, depthWrite: false }))
      disc.position.set(x, y + 0.03, 0.001)
      world.add(disc)
    }

    const meshCache = new Map()
    const geometryFor = meshId => {
      const name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH.value, meshId).replace(/^a\d_/, '')
      if (meshCache.has(name)) return meshCache.get(name)
      const va = model.mesh_vertadr[meshId], vn = model.mesh_vertnum[meshId]
      const fa = model.mesh_faceadr[meshId], fn = model.mesh_facenum[meshId]
      const g = new THREE.BufferGeometry()
      g.setAttribute('position', new THREE.BufferAttribute(Float32Array.from(model.mesh_vert.subarray(3 * va, 3 * (va + vn))), 3))
      g.setIndex(Array.from(model.mesh_face.subarray(3 * fa, 3 * (fa + fn))))
      const flat = g.toNonIndexed()
      flat.computeVertexNormals()
      g.dispose()
      meshCache.set(name, flat)
      return flat
    }

    const materials = new Map()
    const materialFor = rgba => {
      const k = rgba.join(',')
      if (!materials.has(k)) {
        // Desaturate toward the page: the arms are graphite and paper, not product shots.
        const light = rgba[0] > 0.5
        materials.set(k, new THREE.MeshStandardMaterial({ color: light ? 0xe4e4e4 : 0x3c3c3c, roughness: 0.55, metalness: 0.05, flatShading: true }))
      }
      return materials.get(k)
    }

    // A tennis-yellow ball with a white seam, so it reads as a ball (and you can see it spin).
    const ballTexture = () => {
      const c = document.createElement('canvas')
      c.width = 256; c.height = 128
      const g = c.getContext('2d')
      g.fillStyle = '#f5d90a'
      g.fillRect(0, 0, 256, 128)
      g.strokeStyle = '#ffffff'
      g.lineWidth = 7
      g.beginPath()
      for (let x = 0; x <= 256; x += 2) {
        const y = 64 + 38 * Math.sin((x / 256) * Math.PI * 4)
        x ? g.lineTo(x, y) : g.moveTo(x, y)
      }
      g.stroke()
      const t = new THREE.CanvasTexture(c)
      t.colorSpace = THREE.SRGBColorSpace
      return t
    }
    let ballObj = null
    const drawn = []
    for (let g = 0; g < model.ngeom; g++) {
      if (model.geom_group[g] !== 2) continue
      const type = model.geom_type[g]
      let obj
      if (type === mujoco.mjtGeom.mjGEOM_MESH.value) {
        const mat = model.geom_matid[g]
        const rgba = mat >= 0 ? Array.from(model.mat_rgba.subarray(4 * mat, 4 * mat + 4)) : Array.from(model.geom_rgba.subarray(4 * g, 4 * g + 4))
        obj = new THREE.Mesh(geometryFor(model.geom_dataid[g]), materialFor(rgba))
      } else if (type === mujoco.mjtGeom.mjGEOM_SPHERE.value) {
        obj = new THREE.Mesh(new THREE.SphereGeometry(model.geom_size[3 * g], 32, 20), new THREE.MeshStandardMaterial({ map: ballTexture(), roughness: 0.75 }))
        ballObj = obj
      } else continue
      obj.matrixAutoUpdate = false
      world.add(obj)
      drawn.push([g, obj])
    }

    const camera = new THREE.PerspectiveCamera(26, 2, 0.05, 20)
    // Everywhere any part of an arm can go (measured over the routine and every kind
    // of throw, catch and pickup: x -0.55..0.95, y -0.33..0.47, z 0..0.85 m), plus
    // margin. The camera always fits this whole box, so no arm is ever cropped.
    const ENVELOPE = { x: [-0.61, 1.0], y: [-0.38, 0.52], z: [0, 0.89] }
    const corners = []
    for (const x of ENVELOPE.x) for (const y of ENVELOPE.y) for (const z of ENVELOPE.z) corners.push(new THREE.Vector3(x, z, -y)) // MuJoCo → three
    const viewDir = new THREE.Vector3(0.3, 0.36, -1).normalize() // elevated three-quarter view, from the side the arms reach toward
    const fit = () => {
      const w = canvas.clientWidth, h = canvas.clientHeight
      if (!w || !h) return
      renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, w < 600 ? 1.5 : 2))
      renderer.setSize(w, h, false)
      camera.aspect = w / h
      camera.updateProjectionMatrix()
      const target = new THREE.Vector3(0.19, 0.38, -0.25)
      let dist = 3
      const p = new THREE.Vector3(), right = new THREE.Vector3(), up = new THREE.Vector3()
      for (let it = 0; it < 8; it++) {
        camera.position.copy(target).addScaledVector(viewDir, dist)
        camera.lookAt(target)
        camera.updateMatrixWorld()
        let x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9
        for (const c of corners) {
          p.copy(c).project(camera)
          x0 = Math.min(x0, p.x); x1 = Math.max(x1, p.x); y0 = Math.min(y0, p.y); y1 = Math.max(y1, p.y)
        }
        // recentre, then scale the distance so the box fills ~94% of the frame
        const halfH = dist * Math.tan(THREE.MathUtils.degToRad(camera.fov / 2))
        right.setFromMatrixColumn(camera.matrixWorld, 0)
        up.setFromMatrixColumn(camera.matrixWorld, 1)
        target.addScaledVector(right, ((x0 + x1) / 2) * halfH * camera.aspect).addScaledVector(up, ((y0 + y1) / 2) * halfH)
        dist *= Math.max((x1 - x0) / 2, (y1 - y0) / 2) / 0.97
      }
      camera.position.copy(target).addScaledVector(viewDir, dist)
      camera.lookAt(target)
      camera.updateProjectionMatrix()
    }
    // On wide screens the page is one screen tall: the arms take the height that is left.
    const wide = window.matchMedia('(min-width: 56rem)')
    const size = () => {
      if (!wide.matches) { canvas.style.height = ''; return }
      const top = canvas.getBoundingClientRect().top + window.scrollY
      const caption = figure.querySelector('figcaption')?.offsetHeight || 0
      const room = window.innerHeight - top - caption - 16
      canvas.style.height = `${Math.round(Math.max(170, Math.min(420, room)))}px`
    }
    size()
    window.addEventListener('resize', size)
    resizer = new ResizeObserver(fit)
    resizer.observe(canvas)
    fit()


    // --- throw the ball: drag it anywhere, let go to throw; a tap tosses it up ---
    const ray = new THREE.Raycaster()
    const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0.4) // MuJoCo y = 0.4, the plane the arms catch in
    const ndc = new THREE.Vector2()
    const hit = new THREE.Vector3()
    const toWorld = e => {
      const r = canvas.getBoundingClientRect()
      ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1)
      ray.setFromCamera(ndc, camera)
      if (!ray.ray.intersectPlane(plane, hit)) return null
      return [hit.x, -hit.z, hit.y] // three → MuJoCo
    }
    let drag = null
    const caption = figure.querySelector('figcaption')
    const onDown = e => {
      if (e.button > 0) return
      const p = toWorld(e)
      if (!p) return
      drag = { id: e.pointerId, x: e.clientX, y: e.clientY, t0: performance.now(), samples: [[performance.now(), p]] }
      try { canvas.setPointerCapture(e.pointerId) } catch {}
      canvas.classList.add('grabbing')
      sim.grab(p)
    }
    const onMove = e => {
      if (!drag) { if (e.pointerType === 'mouse') sim.lookAt(toWorld(e)); return } // idle arms glance at the pointer
      if (e.pointerId !== drag.id) return
      const p = toWorld(e)
      if (!p) return
      sim.move(p)
      const now = performance.now()
      drag.samples.push([now, p])
      while (drag.samples.length > 2 && now - drag.samples[0][0] > 90) drag.samples.shift()
    }
    const onUp = (e, cancelled) => {
      if (!drag || e.pointerId !== drag.id) return
      const moved = Math.hypot(e.clientX - drag.x, e.clientY - drag.y)
      let v = [0, 0, 0]
      if (!cancelled && moved < 6 && performance.now() - drag.t0 < 350) {
        v = [0, 0, 1.6] // a tap: toss it straight up
      } else if (!cancelled && drag.samples.length > 1) {
        const [t0, a] = drag.samples[0], [t1, b] = drag.samples[drag.samples.length - 1]
        const dt = Math.max(0.016, (t1 - t0) / 1000)
        v = [(b[0] - a[0]) / dt, (b[1] - a[1]) / dt, (b[2] - a[2]) / dt]
      }
      sim.release(v)
      drag = null
      canvas.classList.remove('grabbing')
    }
    if (!window.matchMedia?.('(prefers-reduced-motion: reduce)').matches) {
      canvas.addEventListener('pointerdown', onDown)
      canvas.addEventListener('pointermove', onMove)
      canvas.addEventListener('pointerup', e => onUp(e, false))
      canvas.addEventListener('pointercancel', e => onUp(e, true)) // the page scrolled instead
      canvas.addEventListener('pointerleave', () => sim.lookAt(null))
    } else if (caption) caption.remove()

    const spin = { q: new THREE.Quaternion(), last: null, t: 0 }
    const sync = () => {
      const xp = data.geom_xpos, xm = data.geom_xmat
      for (const [g, obj] of drawn) {
        const p = 3 * g, r = 9 * g
        obj.matrix.set(xm[r], xm[r + 1], xm[r + 2], xp[p], xm[r + 3], xm[r + 4], xm[r + 5], xp[p + 1], xm[r + 6], xm[r + 7], xm[r + 8], xp[p + 2], 0, 0, 0, 1)
        obj.matrixWorldNeedsUpdate = true
      }
      // spin the ball in flight, rolling with its velocity; held, it keeps its pose
      if (ballObj) {
        const now = performance.now()
        const bp = new THREE.Vector3(ballObj.matrix.elements[12], ballObj.matrix.elements[13], ballObj.matrix.elements[14])
        if (spin.last && sim.flying) {
          const dt = Math.max(1e-3, (now - spin.t) / 1000)
          const v = bp.clone().sub(spin.last).divideScalar(dt)
          const axis = new THREE.Vector3(0, 0, 1).cross(v)
          const speed = axis.length()
          if (speed > 1e-4) spin.q.premultiply(new THREE.Quaternion().setFromAxisAngle(axis.normalize(), (speed / 0.02) * dt * 0.6))
        }
        spin.last = bp; spin.t = now
        ballObj.matrix.compose(bp, spin.q, new THREE.Vector3(1, 1, 1))
      }
    }

    const still = window.matchMedia?.('(prefers-reduced-motion: reduce)').matches
    let visible = true
    let last = performance.now()
    observer = new IntersectionObserver(([e]) => { visible = e.isIntersecting; last = performance.now() })
    observer.observe(figure)

    const frame = now => {
      if (disposed) return
      raf = requestAnimationFrame(frame)
      if (!visible || document.hidden) { last = now; return }
      try {
        const dt = Math.min(0.05, (now - last) / 1000)
        last = now
        if (!still) sim.step(dt)
        sync()
        renderer.render(scene, camera)
      } catch (err) {
        fail(err)
      }
    }

    sync()
    renderer.render(scene, camera)
    figure.classList.add('ready')
    if (!still) raf = requestAnimationFrame(frame)
  } catch (err) {
    fail(err)
  }
}
