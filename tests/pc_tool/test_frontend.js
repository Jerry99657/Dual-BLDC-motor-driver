// Execute the shipped page script against a DOM/canvas mock. No browser or hardware.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync('pc_tool/index.html','utf8');
const code=html.match(/<script>([\s\S]*?)<\/script>/)[1],elements=new Map(),requests=[];
class Element {
  constructor(id=''){this.id=id;this.value=id==='window'?'10':id==='scrub'?'1000':'';this.textContent='';this.children=[];this.files=[];this.style={}}
  append(...items){this.children.push(...items)}
  replaceChildren(...items){this.children=items}
  getBoundingClientRect(){return {width:1000,height:480}}
  getContext(){return new Proxy({},{get:(o,key)=>o[key]||(()=>{}),set:(o,key,value)=>(o[key]=value,true)})}
}
const context=vm.createContext({console,Blob,TextDecoderStream,devicePixelRatio:1,
  document:{getElementById:id=>{if(!elements.has(id))elements.set(id,new Element(id));return elements.get(id)},
    createElement:()=>new Element(),createTextNode:text=>text},
  Option:function(text,value){return {text,value}},addEventListener(){},setTimeout(){},
  fetch:async(path,options)=>{requests.push([path,options]);return {ok:true,json:async()=>path==='/api/ports'?{ports:[]}:
    path.startsWith('/api/poll')?{connected:'',status:{},frames:[],received:0,age_ms:null,crc_errors:0,lost:0,resets:0,recording:'',overrun:false}:{ok:true}}}});
async function main(){
  vm.runInContext(code,context);await new Promise(setImmediate);
  assert(requests.every(([,o])=>!o?.method || o.method==='GET'),'Initial UI must not transmit commands');
  vm.runInContext(`add({kind:1,ms:100,epoch:0,values:[['m1.feedback_speed','rad/s',2],['vbus','V',20],['m1.position','rad',null]]});draw();`,context);
  assert.equal(vm.runInContext('series.size',context),3);
  vm.runInContext(`add({kind:4,ms:110,epoch:0,values:[['hso.relative_position','rad',0]]});draw();`,context);
  assert.equal(vm.runInContext('series.size',context),1,'Changing the stream must clear stale channels');
  const rows=['host_time,board_ms,sequence,epoch,packet_kind,field,unit,value'];
  for(let i=0;i<3000;i++)rows.push(`t,${i*10},${i},0,1,m1.feedback_speed,rad/s,${i}`);
  rows.push('t,30000,3000,0,16,status,json,"{""work"":""Single M1""}"');
  context.csvFile=new Blob(['\ufeff'+rows.join('\r\n')+'\r\n']);context.csvFile.name='test.csv';
  await vm.runInContext('loadCsv(csvFile)',context);
  assert(vm.runInContext('replaying',context));
  assert(vm.runInContext('[...series.values()][0].points.length<=2200',context),'Replay memory must be bounded');
  assert.equal(vm.runInContext('latestTime',context),29.99);
  await vm.runInContext(`act(()=>send('M1:Speed:10'))`,context);
  assert(!requests.some(([,o])=>o?.body?.includes('M1:Speed:10')),'History view must not start a live motor');
  await vm.runInContext(`act(()=>send('Stop'))`,context);
  assert(requests.some(([,o])=>o?.body===JSON.stringify({command:'Stop'})));
  console.log('PASS: shipped JavaScript, live unit lanes/invalid values, stream changes, bounded streaming CSV replay, no initial motion command, replay blocks motion while explicit Stop remains available');
}
main().catch(e=>{console.error(e);process.exitCode=1});
