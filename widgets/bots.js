// Little ASCII robots on white tiles, drifting in the page margins and
// winking. Ported from the enigma-theme Claude desktop mod, in black and
// white. They only appear where there is empty margin, so they never sit on
// top of text; on phones there is no margin and there are no robots.

const BOTS = [
  [
    String.raw`    o
   _|_
 .[o o].
  | ~ |
 /|___|\
   d b`,
    String.raw`    o
   _|_
 .[- -].
  | ~ |
 /|___|\
   d b`,
    String.raw`    o
   _|_
 .[^ o].
  | o |
 \|___|/
   d b`,
  ],
  [
    String.raw` .-----.
 | O O |
 |  =  |
 '-----'
  /| |\
  ^   ^`,
    String.raw` .-----.
 | - - |
 |  =  |
 '-----'
  /| |\
  ^   ^`,
    String.raw` .-----.
 | O - |
 |  o  |
 '-----'
  \| |/
  ^   ^`,
  ],
  [
    String.raw`   \_/
  (o.o)
  /[=]\
   / \ `,
    String.raw`   \_/
  (-.-)
  /[=]\
   / \ `,
    String.raw`   \_/
  (o.-)
  \[=]/
   / \ `,
  ],
  [
    String.raw`  ___[]___
 | [o][o] |
 |   __   |
 |________|
   ||  ||
  [__][__]`,
    String.raw`  ___[]___
 | [-][-] |
 |   __   |
 |________|
   ||  ||
  [__][__]`,
    String.raw`  ___[]___
 | [o][-] |
 |   ()   |
 |________|
   ||  ||
  [__][__]`,
  ],
  [
    String.raw` [o_o]
  /|\
  ~~~`,
    String.raw` [-_-]
  /|\
  ~~~`,
    String.raw` [o_-]
  /|\
  ~ ~`,
  ],
]

// rest, blink, rest, wink/gesture, rest — as fractions of each bot's tempo
const SEQ = [[0, 0], [0.55, 1], [0.61, 0], [0.74, 2], [0.94, 0]]
const MIN_GUTTER = 150

export function start(root) {
  if (!root || !window.matchMedia) return
  const still = window.matchMedia('(prefers-reduced-motion: reduce)').matches

  const tiles = BOTS.map((frames, i) => {
    const el = document.createElement('pre')
    el.className = 'bot'
    el.textContent = frames[0]
    el.style.visibility = 'hidden'
    root.appendChild(el)
    return {
      el, frames, frame: 0,
      side: i % 2 ? 1 : -1,
      tempo: [4.2, 5.1, 3.6, 4.7, 2.9][i] * 1000,
      phase: i * 900,
      // slow Lissajous wander inside the bot's margin
      fx: 1 / (17 + i * 4), fy: 1 / (29 + i * 6), ox: i * 1.7, oy: i * 2.3,
      lane: [0.12, 0.3, 0.52, 0.7, 0.88][i],
    }
  })

  let w = 0, h = 0, gutter = 0
  const measure = () => {
    w = document.documentElement.clientWidth
    h = window.innerHeight
    const main = document.querySelector('main')
    const col = main ? main.getBoundingClientRect().width - 32 : 576 // content box, minus padding
    gutter = (w - Math.min(w, col)) / 2
    for (const t of tiles) {
      t.bw = t.el.offsetWidth
      t.bh = t.el.offsetHeight
      t.fits = gutter - 24 >= Math.max(MIN_GUTTER, t.bw + 16)
    }
  }

  const draw = now => {
    for (const t of tiles) {
      if (!t.fits) { t.el.style.visibility = 'hidden'; continue }
      t.el.style.visibility = 'visible'
      const s = now / 1000
      const room = Math.max(0, gutter - t.bw - 32)
      const x0 = t.side < 0 ? 16 : w - gutter + 16
      const x = x0 + room * (0.5 + 0.5 * Math.sin(s * t.fx * 2 * Math.PI + t.ox))
      const yCenter = t.lane * (h - t.bh)
      const y = Math.min(h - t.bh - 8, Math.max(8, yCenter + 0.12 * h * Math.sin(s * t.fy * 2 * Math.PI + t.oy)))
      const bob = still ? 0 : 3 * Math.sin(s * 2.6 + t.ox)
      const tilt = still ? 0 : 3 * Math.sin(s * 0.9 + t.oy)
      t.el.style.transform = `translate(${x.toFixed(1)}px, ${(y + bob).toFixed(1)}px) rotate(${tilt.toFixed(2)}deg)`
      if (!still) {
        const u = ((now + t.phase) % t.tempo) / t.tempo
        let f = 0
        for (const [at, fr] of SEQ) if (u >= at) f = fr
        if (f !== t.frame) { t.frame = f; t.el.textContent = t.frames[f] }
      }
    }
  }

  measure()
  window.addEventListener('resize', () => { measure(); if (still) draw(0) })
  if (still) { draw(0); return }
  const loop = now => {
    if (!document.hidden) {
      try { draw(now) } catch { root.remove(); return }
    }
    requestAnimationFrame(loop)
  }
  requestAnimationFrame(loop)
}
