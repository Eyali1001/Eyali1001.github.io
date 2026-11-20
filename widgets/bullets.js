// "live" bullets: each project's dot lights up from its GitHub repo's last push
// (green and pulsing: within a month; amber: within six months; grey otherwise
// or when GitHub can't be reached). Only runs for ?bullets=live.

export async function light() {
  if (document.documentElement.dataset.bullets !== 'live') return
  for (const li of document.querySelectorAll('.home .projects li')) {
    const repo = [...li.querySelectorAll('a[href^="https://github.com/"]')]
      .map(a => new URL(a.href).pathname.split('/').filter(Boolean))
      .find(p => p.length >= 2)
    if (!repo) { li.classList.add('fresh'); continue } // hosted right here
    const key = `push:${repo[0]}/${repo[1]}`
    try {
      let pushed = sessionStorage.getItem(key)
      if (!pushed) {
        const r = await fetch(`https://api.github.com/repos/${repo[0]}/${repo[1]}`)
        if (!r.ok) continue
        pushed = (await r.json()).pushed_at
        sessionStorage.setItem(key, pushed)
      }
      const days = (Date.now() - new Date(pushed).getTime()) / 864e5
      li.classList.add(days < 31 ? 'fresh' : days < 183 ? 'warm' : 'cold')
      li.title = `last push ${Math.round(days)} days ago`
    } catch {}
  }
}
