// Hover notes work with CSS alone (:hover / :focus). This adds the polish:
// keep each popover inside the viewport, and make tap-to-open work on touch
// screens (first tap opens, a tap elsewhere closes; on a link, the second tap
// follows it).

// Easter eggs on the name. Added by script so the page's <h1> (what search
// engines read) stays just the name.
const EGGS = {
  eyal: '<span lang="he" dir="rtl">אֱיָל</span>: strength. From <a href="https://www.sefaria.org/Psalms.88.5?lang=bi" target="_blank" rel="noopener">Psalms 88:5</a>, “<span lang="he" dir="rtl">כְּגֶבֶר אֵין־אֱיָל</span>”: like a man without strength.',
  merhavi: 'A Hebraization (<span lang="he" dir="rtl">עִבְרוּת</span>) of the Polish family name Markovski.',
}

export function enhance() {
  for (const el of document.querySelectorAll('.egg[data-egg]')) {
    if (!EGGS[el.dataset.egg] || el.querySelector('.pop')) continue
    const pop = document.createElement('span')
    pop.className = 'pop'
    pop.setAttribute('role', 'tooltip')
    pop.innerHTML = EGGS[el.dataset.egg]
    el.appendChild(pop)
    el.tabIndex = 0
  }
  const notes = [...document.querySelectorAll('.note')]
  if (!notes.length) return

  const place = note => {
    const pop = note.querySelector('.pop')
    if (!pop) return
    pop.style.setProperty('--dx', '0px')
    const r = pop.getBoundingClientRect()
    const margin = 12
    const over = r.right - (document.documentElement.clientWidth - margin)
    const under = margin - r.left
    if (over > 0) pop.style.setProperty('--dx', `${-over}px`)
    else if (under > 0) pop.style.setProperty('--dx', `${under}px`)
  }

  const close = except => notes.forEach(n => n !== except && n.classList.remove('open'))

  // With a mouse, cards close the moment the pointer leaves — except cards with
  // something to click inside (the email card, "ping me"), which linger long
  // enough to travel into them.
  const hoverable = window.matchMedia('(hover: hover)').matches
  for (const note of notes) {
    const clickable = !!note.querySelector('.pop a, .pop button')
    if (hoverable && clickable) {
      note.addEventListener('mouseenter', () => {
        clearTimeout(note._closeTimer)
        close(note)
        note.classList.add('open')
      })
      note.addEventListener('mouseleave', () => {
        note._closeTimer = setTimeout(() => note.classList.remove('open'), 450)
      })
    }
    note.addEventListener('mouseenter', () => place(note))
    note.addEventListener('focusin', () => place(note))
    note.addEventListener('click', e => {
      if (e.target.closest('.pop')) return
      const touch = window.matchMedia('(hover: none)').matches
      if (!touch) return
      if (note.matches('.me') && note.classList.contains('open')) { note.classList.remove('open'); return } // tap the photo again to shrink it
      if (!note.classList.contains('open')) {
        e.preventDefault()
        close(note)
        note.classList.add('open')
        place(note)
      }
    })
  }
  document.addEventListener('click', e => { if (!e.target.closest('.note')) close() })
  document.addEventListener('keydown', e => { if (e.key === 'Escape') { close(); document.activeElement?.blur?.() } })
}
