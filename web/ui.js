// Shared UI behaviour for the Layout Lab pages: collapsible sections and panel toggles.
const page = location.pathname.split('/').pop() || 'index.html';
const store = {
  get(k){try{return localStorage.getItem(k)}catch{return null}},
  set(k,v){try{localStorage.setItem(k,v)}catch{}},
};
function setup(){
  document.querySelectorAll('.section').forEach((section,i)=>{
    const h = section.querySelector(':scope > h2:first-child');
    if(!h) return;
    const key = `lab:${page}:sec:${h.textContent.trim()}`;
    if(store.get(key)==='1') section.classList.add('collapsed');
    h.tabIndex = 0; h.setAttribute('role','button');
    const toggle = ()=>{const c=section.classList.toggle('collapsed');h.setAttribute('aria-expanded',String(!c));store.set(key,c?'1':'0')};
    h.setAttribute('aria-expanded',String(!section.classList.contains('collapsed')));
    h.addEventListener('click',toggle);
    h.addEventListener('keydown',e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();toggle()}});
  });
  const ws = document.querySelector('.workspace'), actions = document.querySelector('.top-actions');
  if(ws && actions){
    const box = document.createElement('div'); box.className = 'panel-toggle';
    for(const [cls,label,side,title] of [['l','◧ 설정','left','왼쪽 설정 패널 접기/펼치기'],['r','◨ 정보','right','오른쪽 정보 패널 접기/펼치기']]){
      const b = document.createElement('button'); b.className = 'icon-btn '+cls; b.textContent = label; b.title = title;
      b.addEventListener('click',()=>{ws.classList.toggle('hide-'+side);window.dispatchEvent(new Event('resize'))});
      box.append(b);
    }
    actions.prepend(box);
  }
}
if(document.readyState==='loading') document.addEventListener('DOMContentLoaded',setup); else setup();
