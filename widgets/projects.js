// Project texts: cut off after two lines with "… more", which opens the rest
// in place ("less" closes it). Without this script every text shows in full.

export function wire() {
  const lists = [...document.querySelectorAll('.home .projects')]
  const items = lists.flatMap(l => [...l.querySelectorAll('li')]).filter(li => li.querySelector('.entry .desc') && li.querySelector('.more'))
  if (!items.length) return
  lists.forEach(l => l.classList.add('clamp'))

  // "more" only where the text really is cut off
  const measure = () => {
    for (const li of items) {
      if (li.classList.contains('open')) continue
      const e = li.querySelector('.entry')
      li.classList.toggle('cut', e.scrollHeight > e.clientHeight + 2)
    }
  }
  const still = window.matchMedia('(prefers-reduced-motion: reduce)').matches
  const label = (li, open) => {
    const b = li.querySelector('.more')
    b.textContent = open ? 'less' : 'more'
    b.setAttribute('aria-expanded', String(open))
  }
  // Opening and closing glide: the entry's height eases between its two sizes
  // (the full text stays laid out while it closes, and is cut off at the end).
  const toggle = (li, open) => {
    li._want = open
    li._anim = null
    li.getAnimations?.().forEach(a => a.cancel())
    if (still || !li.animate) {
      li.classList.toggle('open', open)
      label(li, open)
      if (!open) measure()
      return
    }
    const from = li.offsetHeight
    li.classList.toggle('open', open)
    const to = li.offsetHeight
    if (!open) li.classList.add('open') // keep the text whole until the end
    label(li, open)
    li.style.overflow = 'hidden'
    const anim = li.animate([{ height: `${from}px` }, { height: `${to}px` }], { duration: 260, easing: 'cubic-bezier(.25, .7, .25, 1)' })
    li._anim = anim
    const done = () => {
      if (li._anim !== anim) return // a newer toggle took over, or already done
      li._anim = null
      li.style.overflow = ''
      if (!open) { li.classList.remove('open'); measure() }
    }
    anim.onfinish = done
    anim.oncancel = done
    // never stuck half-way, even where animations don't run (e.g. a paused tab)
    setTimeout(() => { if (li._anim === anim) { done(); anim.cancel() } }, 600)
  }

  for (const li of items) {
    li.querySelector('.more').addEventListener('click', () => toggle(li, !(li._want ?? false)))
    // clicking the cut-off text itself opens it too (but links keep working)
    li.querySelector('.entry .desc').addEventListener('click', e => {
      if (e.target.closest('a') || !li.classList.contains('cut') || li._want) return
      toggle(li, true)
    })
  }
  document.addEventListener('keydown', e => { if (e.key === 'Escape') items.forEach(li => li._want && toggle(li, false)) })

  measure()
  if ('ResizeObserver' in window) new ResizeObserver(measure).observe(lists[0].parentElement)
  document.fonts?.ready.then(measure).catch(() => {})
}
