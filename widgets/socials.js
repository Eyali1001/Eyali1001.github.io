// Social links: the email address is assembled only on click (never present
// in the HTML or the live DOM), and the GitHub cards fetch a live line from the
// public API the first time they open. Every failure leaves the static text.

const rev = s => [...s].reverse().join('')

function ago(iso) {
  const s = (Date.now() - new Date(iso).getTime()) / 1000
  const units = [[86400 * 365, 'year'], [86400 * 30, 'month'], [86400 * 7, 'week'], [86400, 'day'], [3600, 'hour'], [60, 'minute']]
  for (const [n, u] of units) if (s >= n) { const k = Math.floor(s / n); return `${k} ${u}${k > 1 ? 's' : ''} ago` }
  return 'just now'
}

async function githubLine(user) {
  const key = `gh:${user}`
  try { const hit = sessionStorage.getItem(key); if (hit) return hit } catch {}
  const ctl = new AbortController()
  const timer = setTimeout(() => ctl.abort(), 4000)
  try {
    const [u, repos] = await Promise.all([
      fetch(`https://api.github.com/users/${user}`, { signal: ctl.signal }).then(r => (r.ok ? r.json() : Promise.reject())),
      fetch(`https://api.github.com/users/${user}/repos?sort=pushed&per_page=1`, { signal: ctl.signal }).then(r => (r.ok ? r.json() : Promise.reject())),
    ])
    let line = `${u.public_repos} public repos`
    if (repos[0]) line += ` · last push: ${repos[0].name}, ${ago(repos[0].pushed_at)}`
    try { sessionStorage.setItem(key, line) } catch {}
    return line
  } finally {
    clearTimeout(timer)
  }
}

export function wire() {
  for (const el of document.querySelectorAll('.social.email')) {
    const address = () => rev(el.dataset.u) + '@' + rev(el.dataset.d)
    const write = e => { e.preventDefault(); e.stopPropagation(); window.location.href = 'mailto:' + address() }
    el.querySelector('.write')?.addEventListener('click', write)
    // On a mouse, clicking the label writes; on touch, the first tap opens the card (notes.js).
    el.querySelector('.label')?.addEventListener('click', e => { if (!window.matchMedia('(hover: none)').matches) write(e) })
    el.addEventListener('keydown', e => { if (e.key === 'Enter' && e.target === el) write(e) })
    const copy = el.querySelector('.copy')
    if (copy) {
      copy.closest('.copy-wrap').hidden = false
      copy.addEventListener('click', async e => {
        e.preventDefault(); e.stopPropagation()
        let ok = false
        try { await navigator.clipboard.writeText(address()); ok = true } catch {}
        if (!ok) {
          // Older/locked-down browsers: copy through a throwaway, off-screen text field.
          const t = document.createElement('textarea')
          t.value = address()
          t.setAttribute('readonly', '')
          t.style.cssText = 'position:fixed;top:-100px;opacity:0'
          document.body.appendChild(t)
          t.select()
          try { ok = document.execCommand('copy') } catch {}
          t.remove()
        }
        copy.textContent = ok ? 'copied ✓' : 'couldn’t copy'
        setTimeout(() => (copy.textContent = 'copy address'), 1800)
      })
    }
  }

  // The GitHub split: pressing it toggles; it folds back on its own after a few
  // seconds (not while the pointer is on it), or when you click elsewhere.
  for (const gh of document.querySelectorAll('.social.gh')) {
    let timer = 0
    let hovering = false
    const isOpen = () => gh.classList.contains('open') || gh.matches(':focus-within')
    const shut = () => {
      clearTimeout(timer)
      if (!isOpen() || gh.classList.contains('closing')) return
      gh.classList.add('closing')
      setTimeout(() => {
        gh.classList.remove('open', 'closing')
        gh.classList.add('merged') // the word "GitHub" fades back in
        setTimeout(() => gh.classList.remove('merged'), 300)
        if (gh.contains(document.activeElement)) document.activeElement.blur()
      }, 220)
    }
    const arm = () => { clearTimeout(timer); timer = setTimeout(() => { if (!hovering) shut(); else arm() }, 4000) }
    const open = () => { gh.classList.remove('closing'); gh.classList.add('open'); arm() }
    // Focus (which opens it) lands on pointerdown, before the click; so decide
    // the toggle from the state it was in when the press began.
    let wasOpen = false
    gh.addEventListener('pointerdown', () => { wasOpen = gh.classList.contains('open') && !gh.classList.contains('closing') })
    gh.addEventListener('click', e => {
      if (e.target.closest('a')) return // a link inside: let it open
      wasOpen ? shut() : open()
    })
    gh.addEventListener('focusin', () => { if (!gh.classList.contains('open')) open() })
    gh.addEventListener('mouseenter', () => { hovering = true })
    gh.addEventListener('mouseleave', () => { hovering = false })
    gh.addEventListener('keydown', e => {
      if ((e.key === 'Enter' || e.key === ' ') && e.target === gh) { e.preventDefault(); gh.classList.contains('open') ? shut() : open() }
    })
    document.addEventListener('click', e => { if (!gh.contains(e.target)) shut() })
    document.addEventListener('keydown', e => { if (e.key === 'Escape') shut() })
  }

  // "Ping me" links point at the contact row: it flashes, one link after another.
  const row = document.getElementById('contact')
  if (row) {
    for (const a of document.querySelectorAll('a.ping')) {
      a.addEventListener('click', e => {
        e.preventDefault(); e.stopPropagation()
        a.closest('.note')?.classList.remove('open')
        if (document.activeElement?.closest?.('.note')) document.activeElement.blur()
        row.classList.remove('pinged')
        void row.offsetWidth // restart the animation on repeat clicks
        row.classList.add('pinged')
        row.scrollIntoView({ block: 'nearest', behavior: 'smooth' })
      })
    }
  }

  for (const pop of document.querySelectorAll('.pop[data-gh]')) {
    const note = pop.closest('.note')
    let done = false
    const load = () => {
      if (done) return
      done = true
      githubLine(pop.dataset.gh).then(line => {
        const span = document.createElement('span')
        span.className = 'gh-live'
        span.textContent = line
        pop.appendChild(span)
      }).catch(() => {})
    }
    note.addEventListener('mouseenter', load)
    note.addEventListener('focusin', load)
    note.addEventListener('touchstart', load, { passive: true })
  }
}
