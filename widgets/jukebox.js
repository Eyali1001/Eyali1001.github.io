// Song links (a.tune[data-yt]): hovering (or tapping) opens a small YouTube
// player under the words. Press play and it keeps going: when you move on, the
// player tucks into the corner, still playing, with a × to close it. Drag its
// title bar to put it anywhere; it then stays where you left it.
//
// The player is created on first use only (nothing loads from YouTube before
// that), uses the no-cookie embed, and is never moved in the page (moving an
// iframe reloads it), only repositioned. Without this script the link simply
// opens the song on YouTube.

export function wire() {
  const tunes = [...document.querySelectorAll('a.tune[data-yt]')]
  if (!tunes.length) return
  const touch = () => window.matchMedia('(hover: none)').matches

  let box = null, frame = null, current = null, engaged = false, state = 'hidden', hideTimer = 0
  let placed = null // { x, y, w } once the visitor has dragged the player somewhere

  const build = tune => {
    box = document.createElement('div')
    box.className = 'jukebox'
    box.setAttribute('role', 'dialog')
    box.setAttribute('aria-label', tune.textContent)
    frame = document.createElement('iframe')
    frame.title = tune.textContent
    frame.allow = 'autoplay; encrypted-media; picture-in-picture'
    frame.referrerPolicy = 'strict-origin-when-cross-origin'
    frame.src = `https://www.youtube-nocookie.com/embed/${encodeURIComponent(tune.dataset.yt)}?rel=0&playsinline=1`
    const bar = document.createElement('div')
    bar.className = 'jukebox-bar'
    const label = document.createElement('span')
    label.textContent = `♪ ${tune.textContent}`
    const close = document.createElement('button')
    close.type = 'button'
    close.className = 'jukebox-close'
    close.setAttribute('aria-label', 'Close the player')
    close.textContent = '×'
    close.addEventListener('click', () => {
      // stop the song: drop the player entirely
      box.remove()
      box = frame = current = null
      engaged = false
      placed = null
      state = 'hidden'
    })
    bar.append(label, close)
    drag(bar)
    box.append(bar, frame)
    document.body.appendChild(box)
    box.addEventListener('mouseenter', () => clearTimeout(hideTimer))
    box.addEventListener('mouseleave', () => leave())
    current = tune
  }

  // Under the words, kept on screen.
  const placeUnder = () => {
    if (!box || !current || state !== 'open') return
    const r = current.getBoundingClientRect()
    const w = box.offsetWidth, h = box.offsetHeight, m = 8
    let x = Math.min(Math.max(m, r.left), window.innerWidth - w - m)
    let y = r.bottom + 6
    if (y + h > window.innerHeight - m) y = Math.max(m, r.top - h - 6)
    box.style.left = `${Math.round(x)}px`
    box.style.top = `${Math.round(y)}px`
  }

  // Wherever the visitor put it, kept on screen.
  const applyPlaced = () => {
    if (!box || !placed) return
    const m = 4
    placed.x = Math.min(Math.max(m, placed.x), window.innerWidth - placed.w - m)
    placed.y = Math.min(Math.max(m, placed.y), window.innerHeight - box.offsetHeight - m)
    state = 'placed'
    box.className = 'jukebox placed'
    Object.assign(box.style, { left: `${placed.x}px`, top: `${placed.y}px`, right: '', bottom: '', width: `${placed.w}px` })
  }
  // Drag the player by its title bar (mouse or touch).
  const drag = bar => {
    bar.addEventListener('pointerdown', e => {
      if (e.button > 0 || e.target.closest('button')) return
      e.preventDefault()
      const r = box.getBoundingClientRect()
      const dx = e.clientX - r.left, dy = e.clientY - r.top
      try { bar.setPointerCapture(e.pointerId) } catch {}
      box.classList.add('dragging')
      engaged = true // moved on purpose: keep it around
      const move = ev => { placed = { x: ev.clientX - dx, y: ev.clientY - dy, w: r.width }; applyPlaced(); box.classList.add('dragging') }
      const up = () => {
        bar.removeEventListener('pointermove', move)
        bar.removeEventListener('pointerup', up)
        bar.removeEventListener('pointercancel', up)
        box?.classList.remove('dragging')
      }
      bar.addEventListener('pointermove', move)
      bar.addEventListener('pointerup', up)
      bar.addEventListener('pointercancel', up)
    })
  }

  const open = tune => {
    clearTimeout(hideTimer)
    if (box && current !== tune) { box.remove(); box = null; engaged = false; placed = null }
    if (!box) build(tune)
    if (placed) { applyPlaced(); return } // it lives where the visitor put it
    state = 'open'
    box.className = 'jukebox open'
    box.style.right = box.style.bottom = ''
    placeUnder()
  }
  const dock = () => {
    if (placed) { applyPlaced(); return }
    state = 'docked'
    box.className = 'jukebox docked'
    box.style.left = box.style.top = ''
  }
  const hide = () => {
    state = 'hidden'
    box.className = 'jukebox'
  }
  // Moving on: a playing song goes to the corner; an untouched player just closes.
  const leave = (delay = 350) => {
    clearTimeout(hideTimer)
    hideTimer = setTimeout(() => {
      if (!box || state !== 'open' || box.matches(':hover')) return
      engaged ? dock() : hide()
    }, delay)
  }

  // A click into the player (play) moves focus into the iframe, and the page
  // loses focus: that is how we know the visitor started it.
  window.addEventListener('blur', () => {
    setTimeout(() => { if (frame && document.activeElement === frame) engaged = true }, 0)
  })

  for (const tune of tunes) {
    tune.addEventListener('mouseenter', () => { if (!touch()) open(tune) })
    tune.addEventListener('mouseleave', () => { if (!touch()) leave() })
    tune.addEventListener('focus', () => open(tune))
    tune.addEventListener('click', e => {
      // the words open the player rather than leaving for YouTube
      e.preventDefault()
      if (state === 'open' && current === tune && touch()) { engaged ? dock() : hide(); return }
      open(tune)
    })
  }
  document.addEventListener('click', e => {
    if (!box || state !== 'open') return
    if (e.target.closest('.jukebox, a.tune')) return
    engaged ? dock() : hide()
  })
  document.addEventListener('keydown', e => { if (e.key === 'Escape' && box && state === 'open') (engaged ? dock() : hide()) })
  window.addEventListener('scroll', placeUnder, { passive: true })
  window.addEventListener('resize', () => { placeUnder(); applyPlaced() })
}
