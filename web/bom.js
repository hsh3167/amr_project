// BOM viewer: reads bom/*.csv from the repository (same origin) and shows prices.
// MISUMI prices are quoted values (kr.misumi-ec.com, VAT excluded); "Estimate" rows are rough and need quotes.
const FILES={suspension:'./bom/suspension_misumi.csv',concept:'./bom/concept_bom.csv'};
const cache={};
const won=n=>Number.isFinite(n)?n.toLocaleString('ko-KR')+'원':'—';
const esc=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));

export function parseCSV(text){
  const rows=[];let row=[],cell='',quoted=false;
  for(let i=0;i<text.length;i++){
    const c=text[i];
    if(quoted){
      if(c==='"'&&text[i+1]==='"'){cell+='"';i++}
      else if(c==='"')quoted=false;
      else cell+=c;
    }else if(c==='"')quoted=true;
    else if(c===','){row.push(cell);cell=''}
    else if(c==='\n'||c==='\r'){
      if(c==='\r'&&text[i+1]==='\n')i++;
      row.push(cell);cell='';if(row.some(v=>v!==''))rows.push(row);row=[];
    }else cell+=c;
  }
  row.push(cell);if(row.some(v=>v!==''))rows.push(row);
  const [head,...body]=rows;
  return body.map(r=>Object.fromEntries(head.map((h,i)=>[h,r[i]??''])));
}

async function load(kind){
  if(!cache[kind]){
    const r=await fetch(FILES[kind],{cache:'no-cache'});
    if(!r.ok)throw Error('BOM 파일을 불러오지 못했습니다: '+FILES[kind]);
    cache[kind]=parseCSV(await r.text());
  }
  return cache[kind];
}

function suspensionView(rows){
  const items=rows.filter(r=>r.Module!=='Total');
  const sum=basis=>items.filter(r=>r['Price basis']===basis).reduce((n,r)=>n+Number(r['Line total KRW ex VAT']||0),0);
  const confirmed=sum('MISUMI price'),estimate=sum('Estimate (not quoted)'),total=confirmed+estimate;
  const checked=items.find(r=>r['Price basis']==='MISUMI price')?.['Price checked']?.slice(0,10)||'';
  const cards=`<div class="bom-cards">
    <div class="bom-card"><span>MISUMI 가격 합계</span><b>${won(confirmed)}</b><small>사이트 조회가 ${esc(checked)} · VAT 별도</small></div>
    <div class="bom-card est"><span>예상 가격 합계</span><b>${won(estimate)}</b><small>표준품·판금 가공 추정 · 견적 필요</small></div>
    <div class="bom-card total"><span>서스펜션 6모듈 예상 총액</span><b>${won(total)}</b><small>VAT 포함 약 ${won(Math.round(total*1.1))}</small></div>
  </div>`;
  const body=items.map(r=>{
    const est=r['Price basis']!=='MISUMI price';
    const src=/^https?:/.test(r.Source)?` <a href="${esc(r.Source)}" target="_blank" rel="noopener noreferrer" title="출처">↗</a>`:'';
    return `<tr class="${est?'est':''}">
      <td>${esc(r.Module)}</td>
      <td><b>${esc(r['Order code'])}</b>${src}<div class="bom-sub">${esc(r.Manufacturer)}</div></td>
      <td class="num">${esc(r.Quantity)}</td>
      <td class="spec-cell">${esc(r['Dimensions or role'])}<div class="bom-sub">${esc(r.Status)}</div></td>
      <td class="num">${r['Unit price KRW ex VAT']===''?'—':won(Number(r['Unit price KRW ex VAT']))}</td>
      <td class="num">${esc(r['Order qty'])}</td>
      <td class="num"><b>${won(Number(r['Line total KRW ex VAT']||0))}</b></td>
      <td>${esc(r['Lead time'])}</td>
      <td><span class="bom-tag ${est?'est':''}">${est?'추정':'MISUMI 가격'}</span><div class="bom-sub">${esc(r['Price checked'])}</div></td>
    </tr>`}).join('');
  return cards+`<div class="bom-table-wrap"><table class="bom-table">
    <thead><tr><th>모듈</th><th>형번 / 품목</th><th class="num">수량</th><th>사양 · 역할 / 상태</th><th class="num">단가</th><th class="num">주문</th><th class="num">금액</th><th>납기</th><th>가격 근거</th></tr></thead>
    <tbody>${body}</tbody>
    <tfoot><tr><td colspan="6">합계 (VAT 별도)</td><td class="num"><b>${won(total)}</b></td><td colspan="2">MISUMI ${won(confirmed)} + 추정 ${won(estimate)}</td></tr></tfoot>
  </table></div>
  <p class="bom-note">X-SWU21-70은 최소 주문 10개 기준입니다. 가격·납기는 수시로 바뀌므로 주문 전에 다시 확인하세요. 추정 행은 견적을 받지 않은 대략값입니다. 원본: <a href="https://github.com/hsh3167/amr_project/blob/main/bom/suspension_misumi.csv" target="_blank" rel="noopener noreferrer">bom/suspension_misumi.csv ↗</a></p>`;
}

function conceptView(rows){
  const known=rows.filter(r=>/^\d+$/.test(r.Cost_KRW)).reduce((n,r)=>n+Number(r.Cost_KRW),0);
  const tbd=rows.filter(r=>!/^\d+$/.test(r.Cost_KRW)).length;
  const body=rows.map(r=>`<tr>
      <td>${esc(r.Category)}</td><td><b>${esc(r.Item)}</b><div class="bom-sub">${esc(r.Supply)}</div></td>
      <td class="num">${esc(r.Qty)} ${esc(r.Unit)}</td>
      <td class="spec-cell">${esc(r.Specification)}<div class="bom-sub">${esc(r.Notes)}</div></td>
      <td>${esc(r.Status)}</td>
      <td class="num">${/^\d+$/.test(r.Cost_KRW)?(r.Cost_KRW==='0'?'0원 (보유품)':won(Number(r.Cost_KRW))):'<span class="bom-tag est">미정</span>'}</td>
    </tr>`).join('');
  return `<div class="bom-cards">
    <div class="bom-card"><span>금액 확인·추정 항목 합계</span><b>${won(known)}</b><small>보유품 0원 포함 · VAT 별도</small></div>
    <div class="bom-card est"><span>가격 미정 항목</span><b>${tbd}개</b><small>판재·전장·배선 등 견적 전</small></div>
  </div>
  <div class="bom-table-wrap"><table class="bom-table">
    <thead><tr><th>구분</th><th>품목</th><th class="num">수량</th><th>사양 / 비고</th><th>상태</th><th class="num">금액</th></tr></thead>
    <tbody>${body}</tbody></table></div>
  <p class="bom-note">개념 BOM은 로봇 전체 구성입니다. 서스펜션 금액은 서스펜션 탭의 예상 총액입니다. 원본: <a href="https://github.com/hsh3167/amr_project/blob/main/bom/concept_bom.csv" target="_blank" rel="noopener noreferrer">bom/concept_bom.csv ↗</a></p>`;
}

export function setupBOM(){
  const dialog=document.getElementById('bomDialog'),content=document.getElementById('bomContent');
  if(!dialog)return;
  let current='suspension';
  async function show(kind){
    current=kind;
    dialog.querySelectorAll('[data-bom]').forEach(b=>b.classList.toggle('active',b.dataset.bom===kind));
    content.innerHTML='<p class="bom-note">불러오는 중…</p>';
    try{const rows=await load(kind);if(current===kind)content.innerHTML=kind==='suspension'?suspensionView(rows):conceptView(rows)}
    catch(e){content.innerHTML=`<p class="bom-note">${esc(e.message)}</p>`}
  }
  document.getElementById('openBom').addEventListener('click',()=>{dialog.showModal();show(current)});
  dialog.querySelectorAll('[data-bom]').forEach(b=>b.addEventListener('click',()=>show(b.dataset.bom)));
  document.getElementById('closeBom').addEventListener('click',()=>dialog.close());
  dialog.addEventListener('click',e=>{if(e.target===dialog)dialog.close()});
  window.amrBOM={load,parseCSV,show};
}
