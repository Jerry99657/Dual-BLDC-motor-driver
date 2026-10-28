// Exercise the actual offline script, including a small-memory configuration.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const code=fs.readFileSync('pc_tool/index.html','utf8').match(/<script>([\s\S]*?)<\/script>/)[1];
function engine(budget){
 class Element{
  constructor(){this.value='';this.textContent='';this.children=[];this.style={};this.dataset={};this.attributes={};this.hidden=false;this.disabled=false}
  append(...x){this.children.push(...x)}replaceChildren(...x){this.children=x}
  setAttribute(k,v){this.attributes[k]=v}addEventListener(k,v){this['on'+k]=v}focus(){}setPointerCapture(){}releasePointerCapture(){}
  getBoundingClientRect(){return {width:1000,height:480}}
  getContext(){return new Proxy({measureText:t=>({width:String(t).length*7})},{get:(o,k)=>o[k]||(()=>{}),set:(o,k,v)=>(o[k]=v,true)})}
 }
 const elements=new Map(),ctx=vm.createContext({console,TextEncoder,TextDecoder,Blob,URL:{},devicePixelRatio:1,performance:{now:()=>100},navigator:{},isSecureContext:true,
  document:{getElementById:id=>{if(!elements.has(id))elements.set(id,new Element());return elements.get(id)},createElement:()=>new Element(),createTextNode:t=>t},
  addEventListener(){},setTimeout(){},clearTimeout(){}});
 const run=s=>vm.runInContext(s,ctx);run(budget?code.replace('const CACHE_MEMORY_BYTES=128*1024*1024;',`const CACHE_MEMORY_BYTES=${budget};`):code);return run;
}
const run=engine();
assert(run(`(()=>{
 let seed=19;const random=()=>((seed=(1664525*seed+1013904223)>>>0)/4294967296),points=new PlotSamples(37);
 const verify=(data,a,b)=>{const actual=plotSummary(data,a,b);let min=Infinity,max=-Infinity,mi=-1,ma=-1,gaps=0;for(let i=a;i<b;i++){const v=plotValueAt(data,i);if(!Number.isFinite(v)){gaps++;continue}if(v<min){min=v;mi=i}if(v>max){max=v;ma=i}}if(actual.min!==min||actual.max!==max||actual.minIndex!==mi||actual.maxIndex!==ma||actual.gaps!==gaps)throw Error(JSON.stringify({actual,min,max,mi,ma,gaps,a,b}));};
 for(let i=0;i<1800;i++){points.push([i*.01,i%13?Math.floor(random()*17)-8:null]);if(points.length>340)points.discard(Math.floor(random()*20));if(i%137===0)points.compact(true);const a=Math.floor(random()*points.length),b=a+Math.floor(random()*(points.length-a+1));verify(points,a,b);const start=Math.min(7,points.length),view=new PlotSampleView(points,start);verify(view,0,view.length);}
 const copy=points.slice(3,points.length-2);verify(copy,0,copy.length);return true;
})()`),'Indexed extrema, ties and gaps match brute force through wrapped heads and resizes');

const work=JSON.parse(run(`JSON.stringify((()=>{
 const p=new PlotSamples();for(let i=0;i<200000;i++)p.push([i*.001,i===123456?900:i===123457?-800:Math.sin(i)]);
 const before=runtimeStats.rawScans,queries=runtimeStats.indexNodes;const s=p.rangeSummary(0,p.length);
 const rangeWork=runtimeStats.rawScans-before;const context={beginPath(){},moveTo(){},lineTo(){},stroke(){}};
 const start=runtimeStats.rawScans;strokePlotSamples(context,p,0,p.length,0,200,0,800,0,500,-800,900,1);
 return {rangeWork,rangeNodes:runtimeStats.indexNodes-queries,denseRaw:runtimeStats.rawScans-start,points:p.length,min:s.min,max:s.max};
})())`));
assert.equal(work.min,-800);assert.equal(work.max,900);assert(work.rangeWork<64);assert(work.denseRaw<work.points/4,JSON.stringify(work));

const low=engine(128*1024);
assert(low(`(()=>{
 $('window').value='100';$('window').onchange();for(let i=0;i<2000;i++)add({kind:1,epoch:0,ms:i*10,values:[['budget.speed','rad/s',i]]});setPlotCount(3);for(let i=0;i<3;i++){plots[i].control('window').value='100';plots[i].control('window').onchange();assignPlotChannel(i,'budget.speed|rad/s',true)}
 if(!plots[0].freeze())throw Error('First snapshot refused');globalThis.oldSnapshot=plots[0].view.frozen;
 add({kind:1,epoch:0,ms:20000,values:[['budget.speed','rad/s',2000]]});if(!plots[1].freeze())throw Error('Second snapshot refused');
 add({kind:1,epoch:0,ms:20010,values:[['budget.speed','rad/s',2001]]});if(plots[2].freeze()!==false)throw Error('Over-budget snapshot accepted');
 return plots[0].view.frozen===oldSnapshot&&!plots[2].view.frozen&&cacheMemory().totalBytes<=CACHE_MEMORY_BYTES&&$('error').textContent.includes('预算');
})()`),'Refused pause leaves existing snapshots and live reception intact');
assert(low(`(()=>{
 for(const p of plots)p.resume();pauseAllPlots();const stores=new Set(plots.map(p=>sampleStorage(p.view.frozen.get('budget.speed|rad/s').points)));
 if(stores.size!==1)throw Error('Simultaneous snapshots were copied repeatedly');const end=plots[0].bounds().hi,first=plots[0].view.frozen.get('budget.speed|rad/s').points.at(-1)[1];
 for(let i=2002;i<9000;i++)add({kind:1,epoch:0,ms:i*10,values:[['budget.speed','rad/s',i]]});
 return plots.every(p=>p.bounds().hi===end)&&plots[0].view.frozen.get('budget.speed|rad/s').points.at(-1)[1]===first&&cacheMemory().totalBytes<=CACHE_MEMORY_BYTES;
})()`),'Shared snapshots remain immutable under wrapped live data and global budget pressure');
const atomic=engine(64*1024);
assert(atomic(`(()=>{
 $('window').value='100';$('window').onchange();for(let i=0;i<1000;i++)add({kind:1,epoch:0,ms:i*10,values:[['budget.a','A',i],['budget.b','V',i]]});setPlotCount(2);assignPlotChannel(0,'budget.a|A',true);assignPlotChannel(1,'budget.b|V',true);
 return pauseAllPlots()===false&&plots.every(p=>p.view.frozen===null)&&cacheMemory().totalBytes<=CACHE_MEMORY_BYTES;
})()`),'Pause-all preflight is atomic when all requested snapshots do not fit');
console.log('PASS: real script performance: exact block extrema/nulls/ties across ring wrap/resize/views/copies; indexed dense work '+JSON.stringify(work)+'; shared immutable snapshots, global allocated-byte budget, pause refusal and atomic pause-all');
