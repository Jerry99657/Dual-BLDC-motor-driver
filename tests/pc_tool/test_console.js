// Actual single-file script + model Web Serial/DOM. No browser or hardware.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync('pc_tool/index.html','utf8');
assert(!/<script[^>]+src=|<link[^>]+href=|fetch\(/i.test(html),'No external runtime dependency');
const code=html.match(/<script>([\s\S]*?)<\/script>/)[1],elements=new Map(),writes=[],timers=new Map(),downloads=[];
let timerId=0,serialOptions=null,portClosed=0,serialReader;
class Element{
 constructor(id=''){this.id=id;this.value=({window:'10',scrub:'1000',axis:'M1',motorAxis:'M1',action:'Speed'})[id]||'';this.textContent='';this.children=[];this.files=[];this.style={};this.dataset={};this.disabled=false;this.hidden=false;this.attributes={}}
 append(...items){this.children.push(...items)}replaceChildren(...items){this.children=items}
 setAttribute(name,value){this.attributes[name]=value}
 addEventListener(name,fn){this['on'+name]=fn}
 focus(){}setPointerCapture(){}releasePointerCapture(){}
 getBoundingClientRect(){return {width:1000,height:480}}
 getContext(){return new Proxy({measureText:text=>({width:String(text).length*7})},{get:(o,k)=>o[k]||(()=>{}),set:(o,k,v)=>(o[k]=v,true)})}
 click(){if(this.download)downloads.push(this.download);else this.onclick?.()}
}
const context=vm.createContext({console,Blob,TextEncoder,TextDecoder,TextDecoderStream,Uint8Array,DataView,ArrayBuffer,devicePixelRatio:1,
 isSecureContext:true,performance:{now:()=>100},confirm:()=>true,navigator:{serial:{requestPort:async()=>fakePort}},
 document:{getElementById:id=>{if(!elements.has(id))elements.set(id,new Element(id));return elements.get(id)},createElement:()=>new Element(),createTextNode:t=>t},
 URL:{createObjectURL:()=>`blob:test`,revokeObjectURL(){}},
 addEventListener(){},setTimeout:(fn,ms)=>{const id=++timerId;timers.set(id,{fn,ms});return id},clearTimeout:id=>timers.delete(id)});
const evaluate=s=>vm.runInContext(s,context);
function flush(){return new Promise(setImmediate)}
function crc(data){let c=65535;for(const b of data){c^=b<<8;for(let i=0;i<8;i++)c=((c<<1)^((c&32768)?0x1021:0))&65535}return c}
let sequence=200,boardMs=5000,autoReply=true,replyResult=1,hangWrite=false,rejectWrite;
const rpcBytes=fs.readFileSync('build/console-r87/rpc-fixture.bin'),base=Buffer.from(rpcBytes.subarray(0,106));
const rowBase=fs.readFileSync('build/console-r88/rpc-row-fixture.bin');
const boardReply=Buffer.from(base);
function reply(id,command){
 if(replyResult===1){
  let cfg=boardReply.readUInt32LE(52);const work=['SingleM1','SingleM2','Independent','SyncSpeed','SyncPosition','MasterM1','MasterM2'].indexOf(command.slice(5));
  if(command.startsWith('Work:')&&work>=0){cfg=(cfg&~7)|work;boardReply[19]=work===1||work===5?1:0}
  const model=command.match(/^Motor([12]):(2804|4015|ESC|Custom[123])$/);
  if(model){const shift=model[1]==='1'?24:26;
   if(model[2].startsWith('Custom'))cfg=(cfg&~(3<<shift))|(Number(model[2].at(-1))<<shift);
   else if(model[2]!=='ESC')cfg&=~(3<<shift);
   if(model[1]==='1'){if(model[2]==='ESC')cfg|=32768;else{cfg&=~32768;if(!model[2].startsWith('Custom'))boardReply.writeUInt32LE((boardReply.readUInt32LE(32)&~(1<<24))|(model[2]==='4015'?1<<24:0),32)}}else if(!model[2].startsWith('Custom'))cfg=(cfg&~16)|(model[2]==='4015'?16:0)}
  if(command.startsWith('Feedback:'))cfg=(cfg&~(8|16384))|({Encoder:0,HSO:8,SixStep:16384}[command.slice(9)]||0);
  if(/^Focus:M[12]$/.test(command))boardReply[19]=Number(command.at(-1))-1;
  boardReply.writeUInt32LE(cfg,52);
 }
 const frame=Buffer.from(boardReply);frame.writeUInt16LE(sequence++&65535,4);frame.writeUInt32LE(boardMs+=10,8);frame.writeUInt16LE(id,12);frame.writeInt8(replyResult,14);
 frame.writeUInt16LE(crc(frame.subarray(0,-2)),frame.length-2);return new Uint8Array(frame)}
function rowReply(id,command){const frame=Buffer.from(rowBase),m=command.match(/^GetRow:(\d+),(\d+),(\d+)$/);frame.writeUInt16LE(sequence++&65535,4);frame.writeUInt32LE(boardMs+=10,8);frame.writeUInt16LE(id,12);frame[16]=Number(m[1]);frame[17]=Number(m[2]);frame[18]=Number(m[3]);frame.writeUInt16LE(crc(frame.subarray(0,-2)),frame.length-2);return new Uint8Array(frame)}
function inject(bytes){serialReader.push(bytes)}
const fakePort={open:async options=>{serialOptions=options},close:async()=>{portClosed++},
 writable:{getWriter:()=>({write:async bytes=>{const text=new TextDecoder().decode(bytes).trim();writes.push(text);if(hangWrite)return new Promise((_,reject)=>{rejectWrite=reject});const m=text.match(/^@(\d+):(.*)$/);if(autoReply&&m)inject((m[2].startsWith('GetRow:')?rowReply:reply)(Number(m[1]),m[2]))},abort:async()=>{rejectWrite?.(Error('aborted'));rejectWrite=null},releaseLock(){}})},
 readable:{getReader:()=>{let waiting=null,queue=[];serialReader={read:()=>queue.length?Promise.resolve(queue.shift()):new Promise(resolve=>{waiting=resolve}),push:bytes=>{if(waiting){const f=waiting;waiting=null;f({value:bytes,done:false})}else queue.push({value:bytes,done:false})},cancel:async()=>{if(waiting){const f=waiting;waiting=null;f({done:true})}},releaseLock(){}};return serialReader}}};
const options=id=>elements.get(id).children.map(x=>x.value);
const choose=(id,value)=>evaluate(`$('${id}').value=${JSON.stringify(value)};$('${id}').onchange()`);
async function controlLinkage(){
 await evaluate("send('Motor2:4015')");const before=writes.length;
 choose('workMode','SingleM2');
 assert.deepEqual(options('motorAxis'),['M2']);assert.deepEqual(options('axis'),['M2']);
 assert.deepEqual(options('driveMode'),['Encoder']);assert.deepEqual(options('motorModel'),['2804','4015']);
 assert.equal(elements.get('motorModel').value,'4015');assert(elements.get('setDrive').hidden);
 assert(elements.get('apply').disabled);assert.equal(writes.length,before,'Selector previews send no commands');
 await assert.rejects(evaluate('sendRunTarget()'),/先应用工作模式/);
 await assert.rejects(evaluate('applyMotorModel()'),/先应用工作模式/);
 await evaluate('applyWorkMode()');assert.equal(evaluate('boardConfig.work'),1);
 await assert.rejects(evaluate('applyDriveMode()'),/单 M1/);
 evaluate("$('motorModel').value='ESC'");await assert.rejects(evaluate('applyMotorModel()'),/不支持/);
 choose('motorAxis','M2');evaluate("$('action').value='Throttle';$('target').value='20'");
 await assert.rejects(evaluate('sendRunTarget()'),/不支持/);
 choose('action','Speed');evaluate("$('target').value='12'");await evaluate('sendRunTarget()');assert(writes.at(-1).endsWith(':M2:Speed:12'));
 choose('workMode','SingleM1');await evaluate('applyWorkMode()');
 choose('motorModel','4015');await evaluate('applyMotorModel()');
 assert.deepEqual(options('driveMode'),['Encoder','HSO','SixStep']);
 choose('driveMode','HSO');assert.deepEqual(options('action'),['Speed']);assert(elements.get('apply').disabled);
 await assert.rejects(evaluate('sendRunTarget()'),/先应用驱动/);await evaluate('applyDriveMode()');
 evaluate("$('action').value='Angle'");await assert.rejects(evaluate('sendRunTarget()'),/不支持/);
 choose('driveMode','SixStep');assert.deepEqual(options('action'),['Speed','Throttle']);
 choose('motorModel','ESC');assert.deepEqual(options('action'),['Throttle']);
 await evaluate('applyDriveMode()');assert.equal(elements.get('motorModel').value,'ESC','Drive ACK preserves an unapplied model draft');
 assert(elements.get('apply').disabled);await assert.rejects(evaluate('sendRunTarget()'),/先应用电机型号/);
 await evaluate('applyMotorModel()');assert(!elements.get('apply').disabled);
 evaluate("$('target').value='20'");await evaluate('sendRunTarget()');assert(writes.at(-1).endsWith(':M1:Throttle:20'));
 evaluate("$('target').value='101'");await assert.rejects(evaluate('sendRunTarget()'),/无效/);
 choose('driveMode','Encoder');assert.equal(elements.get('motorModel').value,'4015','Leaving ESC restores the underlying motor profile');
 await evaluate('applyDriveMode()');assert(!elements.get('apply').disabled);
 for(const [work,actions,axes]of [['Independent',['Speed','Angle','Torque'],['M1','M2']],['SyncSpeed',['Speed'],['M1','M2']],['SyncPosition',['Angle'],['M1','M2']],['MasterM1',[],['M2']],['MasterM2',[],['M1']]]){
  choose('workMode',work);assert.deepEqual(options('action'),actions);assert.deepEqual(options('axis'),axes);
  await evaluate('applyWorkMode()');assert.deepEqual(options('driveMode'),['Encoder']);assert(!options('motorModel').includes('ESC'));
  if(work.startsWith('Master')){
   assert(!elements.get('follow').hidden&&elements.get('runTargetRow').hidden);
   choose('motorAxis',work==='MasterM1'?'M1':'M2');assert.equal(elements.get('axis').value,axes[0],'A passive master is never a Run axis');
   await assert.rejects(evaluate('sendRunTarget()'),/Follow/);await evaluate('startFollow()');assert(writes.at(-1).endsWith(':Follow'));
  }else{
   assert(elements.get('follow').hidden);choose('motorAxis','M2');assert.equal(elements.get('axis').value,'M2');assert.equal(elements.get('motorModel').value,'4015');
   evaluate("$('target').value='10'");await evaluate('sendRunTarget()');assert(writes.at(-1).endsWith(':M2:'+actions[0]+':10'));
  }
 }
 choose('workMode','SingleM2');replyResult=-1;await assert.rejects(evaluate('applyWorkMode()'),/板端拒绝/);replyResult=1;
 assert.equal(evaluate('boardConfig.work'),6);assert(elements.get('apply').disabled);assert(/尚未应用/.test(elements.get('workState').textContent));
 evaluate("$('target').value='42'");await evaluate("send('App:0,0,70')");assert.equal(elements.get('workMode').value,'SingleM2');assert.equal(elements.get('target').value,'42');
 await evaluate("send('Get')");assert.equal(elements.get('workMode').value,'MasterM2');assert.deepEqual(options('axis'),['M1']);
 choose('workMode','Independent');await evaluate('applyWorkMode()');
 console.log('PASS: linked work/motor/drive/Run selectors, M2 FOC-only, independent/sync/Follow, HSO/six-step/ESC actions, draft ACK/readback and rejection gates without automatic motion');
}
function plotRegressions(){
 const before=writes.length;
 evaluate(`selected.clear();clear();for(let i=0;i<2500;i++)add({kind:1,epoch:0,ms:i*10,values:[['speed','rad/s',Math.sin(i/30)],['position','rad',i/100],['bus','V',12+.02*Math.sin(i)]]});addAllPlotChannels();draw()`);
 assert.equal(evaluate('plotLayout.panels.length'),1);assert.equal(evaluate('plotLayout.panels[0].items.length'),3);
 assert(evaluate('plotLayout.panels[0].lo<0 && plotLayout.panels[0].hi>=24.99'),'Different units share one numerical Y range');
 assert.equal(evaluate('[...series.values()][0].points.length'),1001);assert(Math.abs(evaluate('plotBounds().hi-plotBounds().lo')-10)<1e-9);
 evaluate(`$('plotAxis').value='x';(()=>{const l=plotLayout,p=l.panels[0];$('plot').onwheel({clientX:(l.left+l.right)/2,clientY:(p.top+p.bottom)/2,deltaY:-120,deltaMode:0,preventDefault(){}})})()`);
 assert(evaluate('plotView.span<10 && plotView.end===null && plotView.frozen===null && plotView.offset<0 && plotView.y.size===0'));
 const end=evaluate('plotLayout.end'),offset=evaluate('plotView.offset');
 evaluate(`for(let i=2500;i<5000;i++)add({kind:1,epoch:0,ms:i*10,values:[['speed','rad/s',0],['position','rad',i/100],['bus','V',12]]});draw()`);
 assert.equal(evaluate('plotBounds().hi'),49.99);assert(Math.abs(evaluate('plotLayout.end')-end-25)<1e-9);
 assert(Math.abs(evaluate('plotView.offset')-offset)<1e-9,'Zoom keeps a live relative window as data arrives');
 evaluate('followPlot(true)');assert(evaluate('plotView.frozen===null && plotView.end===null && !paused'));
 evaluate(`$('plotAxis').value='xy';(()=>{const l=plotLayout,p=l.panels[0];$('plot').onwheel({clientX:l.left-10,clientY:(p.top+p.bottom)/2,deltaY:-120,deltaMode:0,preventDefault(){}})})()`);
 assert(evaluate('plotView.frozen===null && plotView.end===null && plotView.span===10 && plotView.y.size===1'),'Y-axis zoom keeps live X following');
 evaluate('followPlot(true)');
 evaluate(`$('plotAxis').value='x';(()=>{const l=plotLayout,p=l.panels[0],x=(l.left+l.right)/2,y=(p.top+p.bottom)/2;$('plot').onpointerdown({button:0,pointerId:1,clientX:x,clientY:y,preventDefault(){}});$('plot').onpointermove({clientX:x+100,clientY:y});$('plot').onpointerup({pointerId:1})})()`);
 assert(evaluate('plotView.offset<0 && plotView.end===null && plotView.frozen===null && plotView.drag===null'));
 evaluate(`$('plotZoomIn').onclick();scrubPlot(0)`);assert.equal(elements.get('scrub').value,'0');
 evaluate(`$('pause').onclick()`);assert(evaluate('paused && plotView.frozen!==null'));
 const frozenEnd=evaluate('plotBounds().hi'),frozenView=evaluate('plotLayout.end');
 evaluate(`for(let i=5000;i<8000;i++)add({kind:1,epoch:0,ms:i*10,values:[['speed','rad/s',0],['position','rad',i/100],['bus','V',12]]});draw()`);
 assert.equal(evaluate('plotBounds().hi'),frozenEnd);assert.equal(evaluate('plotLayout.end'),frozenView,'Only explicit Pause freezes an immutable snapshot while recording/receiving continues');
 evaluate(`$('pause').onclick()`);assert(evaluate('!paused && plotView.end===null && plotView.frozen===null'));
 evaluate(`followPlot(true);$('plotTool').value='box';$('plotAxis').value='xy';(()=>{const l=plotLayout,p=l.panels[0],x=l.left+50,y=p.top+20;$('plot').onpointerdown({button:0,pointerId:2,clientX:x,clientY:y,preventDefault(){}});$('plot').onpointermove({clientX:x+200,clientY:p.bottom-20});$('plot').onpointerup({pointerId:2})})()`);
 assert(evaluate('plotView.span<10 && plotView.y.size===1 && plotView.end===null && plotView.frozen===null'));
 assert.equal(evaluate('plotLayout.panels.length'),1);assert(!elements.get('plotAutoY').checked,'One manual Y range applies to all curves in this canvas');
 evaluate(`followPlot(true);$('plot').onkeydown({key:'-',preventDefault(){}})`);assert(evaluate('plotView.span>10'));
 evaluate(`$('plot').onkeydown({key:'Home',preventDefault(){}})`);assert(evaluate('plotView.span===10 && plotView.y.size===0 && plotView.end===null'));
 evaluate(`$('plot').onkeydown({key:'ArrowRight',preventDefault(){}})`);
 assert(evaluate('plotView.offset>0 && plotLayout.end>latestTime && plotLayout.start<latestTime && plotView.frozen===null'),'Live view can leave empty space after the newest point');
 evaluate(`$('plotTime').value='board';$('plotTime').onchange()`);assert(!evaluate('plotLayout.relative'));
 evaluate(`$('plotTime').value='relative';$('plotTime').onchange();followPlot(true)`);assert(evaluate('plotLayout.relative && plotLayout.origin===latestTime'));
 evaluate(`$('plotZoomIn').onclick()`);const rangeInitial=evaluate('plotView.span');
 const rangeEnd=evaluate('plotLayout.end-latestTime');
 evaluate(`$('plotBufferStart').onkeydown({key:'ArrowLeft',preventDefault(){}})`);
 assert(evaluate('plotView.span')>rangeInitial&&evaluate('plotView.frozen===null'));assert(Math.abs(evaluate('plotLayout.end-latestTime')-rangeEnd)<1e-8,'Left endpoint keeps the right endpoint fixed');
 const rangeStart=evaluate('plotLayout.start-latestTime');
 evaluate(`$('plotBufferEnd').onkeydown({key:'ArrowLeft',preventDefault(){}})`);
 assert(Math.abs(evaluate('plotLayout.start-latestTime')-rangeStart)<1e-8,'Right endpoint keeps the left endpoint fixed');
 const rangeSpan=evaluate('plotView.span');evaluate(`$('plotBufferMove').onkeydown({key:'ArrowLeft',preventDefault(){}})`);
 assert.equal(evaluate('plotView.span'),rangeSpan,'Green midpoint translates without scaling');
 evaluate(`$('window').value='0.125';$('window').onchange()`);assert(evaluate('plotView.span===.125 && plotView.cacheSeconds===.125 && plotView.frozen===null'));
 for(const invalid of ['',0,-1,'NaN','Infinity',86401]){elements.get('window').value=String(invalid);assert.equal(evaluate(`$('window').onchange()`),false);assert(evaluate('plotView.cacheSeconds===.125 && plotView.span===.125'))}
 evaluate('followPlot(true)');assert.equal(elements.get('window').value,'0.125');assert(elements.get('plotPresetError').hidden);
 evaluate(`$('window').value='30.5';$('window').onchange()`);assert(evaluate('plotView.span===30.5 && Number.isFinite(plotLayout.end)'));
 evaluate(`$('window').value='10';$('window').onchange();followPlot(true)`);
 assert.equal(evaluate("plotValue(16777217)"),'16777217');
 assert.notEqual(evaluate('plotNumber(16777217,.1)'),evaluate('plotNumber(16777217.1,.1)'));
 assert(evaluate('plotNearest([[1,0],[2,null],[3,4]],2)[1]===null && plotNearest([[1,0]],2)===null'));
 evaluate(`freezePlot();add({kind:4,epoch:0,ms:60000,values:[['flux','Wb',.01]]});draw()`);assert(evaluate('plotView.frozen===null && plotView.end===null && plotView.y.size===0'));
 evaluate(`clear();$('plotTool').value='pan';$('plotAxis').value='xy'`);
 assert.equal(evaluate('plotLayout.panels.length'),1,'An empty canvas still shows exactly one XY pair');
 assert.equal(writes.length,before,'Chart interaction sends no commands');
 console.log('PASS: single XY pair, live zoom/pan/box/axes with relative offset and future margin, relative/board time, explicit-pause immutable history, pause/resume, keyboard reset, null gaps, precise counters and no motor commands');
}
function multiPlotRegressions(){
 const before=writes.length;
 evaluate(`selected.clear();clear();for(let i=0;i<1000;i++)add({kind:1,epoch:0,ms:i*20,values:[['multi.speed','rad/s',Math.sin(i/20)],['multi.position','rad',i/20],['multi.current','A',.2],['multi.bus','V',12]]});addAllPlotChannels();draw()`);
 assert.equal(evaluate('setPlotCount(3)'),true);assert.equal(elements.get('plotCanvases').children.length,3);
 assert(evaluate('plots[0].selected.size===4 && plots[1].selected.size===0 && plots[2].selected.size===0'));
 assert.equal(evaluate('setPlotCount(0)'),false);assert.equal(evaluate('setPlotCount(7)'),false);
 assert.equal(evaluate(`assignPlotChannel(1,'multi.speed|rad/s',true)`),true);
 assert(evaluate(`plots[0].selected.has('multi.speed|rad/s') && plots[1].selected.has('multi.speed|rad/s') && activePlotIndex===1 && selected===plots[1].selected`));
 evaluate(`assignPlotChannel(1,'multi.speed|rad/s',true)`);assert.equal(evaluate('plots[1].selected.size'),1);
 assert.equal(evaluate(`assignPlotChannel(2,'<script>bad</script>',true)`),false);
 evaluate(`(()=>{const p=plots[1],l=p.layout,g=l.panels[0];p.control('plot').onwheel({clientX:(l.left+l.right)/2,clientY:(g.top+g.bottom)/2,deltaY:-120,deltaMode:0,preventDefault(){}})})()`);
 assert(evaluate('plots[1].view.span<10 && plots[1].view.frozen===null && plots[1].view.offset<0 && plots[0].view.span===10 && plots[0].view.offset===0'));
 evaluate(`assignPlotChannel(2,'multi.bus|V',true)`);assert.equal(evaluate('activePlotIndex'),2);
 evaluate(`plots[2].root.ondrop({preventDefault(){},stopPropagation(){},dataTransfer:{getData:type=>type===CHANNEL_MIME?'multi.position|rad':''}})`);
 assert(evaluate(`plots[2].selected.has('multi.position|rad') && plots[0].selected.has('multi.position|rad')`));
 assert(evaluate(`plots[2].layout.panels.length===1 && plots[2].layout.panels[0].items.length===2 && plots[2].layout.panels[0].unit==='数值 · 单位见通道'`));
 evaluate(`plots[2].view.y.set('__xy',{lo:0,hi:.01});plots[2].draw();assignPlotChannel(2,'multi.current|A',true)`);
 assert(evaluate('plots[2].view.y.size===0 && plots[2].layout.panels[0].hi>=49.95'),'Adding a new curve refits shared Y without splitting axes');
 evaluate(`assignPlotChannel(2,'multi.current|A',false)`);
 evaluate(`plots[2].control('plotZoomIn').onclick();scrubPlot(0)`);assert(evaluate('plots[2].view.offset<0 && plots[2].view.frozen===null && plots[0].view.offset===0'));
 evaluate('setPlotCount(1)');assert.equal(evaluate('activePlotIndex'),0);assert(evaluate('plots[1].root.hidden && plots[2].root.hidden'));
 evaluate('setPlotCount(3)');assert(evaluate(`plots[1].selected.has('multi.speed|rad/s') && plots[2].selected.size===2 && !plots[2].root.hidden`));
 evaluate(`assignPlotChannel(1,'multi.speed|rad/s',false)`);assert(evaluate(`plots[0].selected.has('multi.speed|rad/s') && !plots[1].selected.size`));
 const otherBindings=evaluate('JSON.stringify(plots.slice(1,3).map(p=>[...p.selected]))');
 evaluate(`activatePlot(0);$('plotAddAll').onclick()`);assert.equal(evaluate('plots[0].selected.size'),4);
 assert.equal(evaluate('JSON.stringify(plots.slice(1,3).map(p=>[...p.selected]))'),otherBindings,'Add all never reallocates other canvases');
 assert(evaluate('plots.slice(0,3).every(p=>p.layout.panels.length===1)'),'Canvas count, not channel unit, determines XY pair count');
 evaluate('pauseAllPlots()');assert(evaluate('paused && plots.slice(0,3).every(p=>p.view.frozen!==null)'));
 evaluate('pauseAllPlots()');assert(evaluate('!paused && plots.slice(0,3).every(p=>p.view.frozen===null && p.view.end===null)'));
 evaluate('setPlotCount(6)');assert.equal(evaluate('plots.length'),6);assert(elements.get('plotAdd').disabled);
 const assigned=evaluate(`[...plots[0].selected].join(',')`);
 evaluate(`add({kind:2,epoch:1,ms:100,values:[['other.speed','rad/s',1]]});draw()`);
 assert.equal(evaluate(`[...plots[0].selected].join(',')`),assigned,'Data reset preserves explicit bindings while clearing old samples');
 assert(evaluate('plots.every(p=>p.view.frozen===null && p.view.end===null && p.view.y.size===0)'));
 evaluate('setPlotCount(1);activatePlot(0);selected.clear();clear()');
 assert.equal(writes.length,before,'Canvas configuration, binding and drag/drop never send commands');
 console.log('PASS: 1..6 independent XY pairs, empty add, mixed-unit copied/deduplicated drops, shared-Y refit, per-canvas zoom/history, hidden restore, removal and bulk-add isolation, pause-all, retained bindings on source reset and zero commands');
}
function cacheRegressions(){
 const before=writes.length;
 evaluate(`for(const p of plots){p.view.cacheSeconds=p.view.span=10;p.selected.clear()}setPlotCount(1);clear();$('window').value='100';$('window').onchange();
  for(let i=0;i<=15000;i++)add({kind:1,epoch:0,ms:i*10,values:[['retention.speed','rad/s',i%97?i:null]]});addAllPlotChannels();draw()`);
 assert.equal(evaluate("series.get('retention.speed|rad/s').points.length"),10001,'100 s at 100 Hz retains original samples beyond 2200');
 assert(evaluate("series.get('retention.speed|rad/s').points.at(0)[0]===50&&series.get('retention.speed|rad/s').points.at(-1)[0]===150"));
 assert(evaluate("plotNearest(series.get('retention.speed|rad/s').points,50.44)[1]===null"),'Null gaps survive Float64 storage');
 evaluate(`setPlotCount(2);assignPlotChannel(1,'retention.speed|rad/s',true)`);
 assert(evaluate('plots[0].bounds().hi-plots[0].bounds().lo===100&&plots[1].bounds().hi-plots[1].bounds().lo===10'),'Shared channels expose each canvas duration');
 evaluate(`plots[0].control('window').value='20';plots[0].control('window').onchange()`);
 assert.equal(evaluate("series.get('retention.speed|rad/s').points.length"),2001,'Shrinking the longest duration immediately evicts older live samples');
 evaluate(`plots[0].control('window').value='100';plots[0].control('window').onchange()`);
 assert.equal(evaluate("series.get('retention.speed|rad/s').points.length"),2001,'Increasing duration cannot recreate discarded history');
 evaluate(`for(let i=15001;i<=25000;i++)add({kind:1,epoch:0,ms:i*10,values:[['retention.speed','rad/s',i]]});draw()`);
 assert.equal(evaluate("series.get('retention.speed|rad/s').points.length"),10001);
 evaluate(`plots[1].control('window').value='120';plots[1].control('window').onchange();setPlotCount(1);
  for(let i=25001;i<=40000;i++)add({kind:1,epoch:0,ms:i*10,values:[['retention.speed','rad/s',i]]});draw()`);
 assert.equal(evaluate("series.get('retention.speed|rad/s').points.length"),12001,'Hidden canvases retain their configured history for restoration');
 assert.equal(evaluate('plots[0].bounds().hi-plots[0].bounds().lo'),100);
 evaluate('setPlotCount(2)');assert.equal(evaluate('plots[1].bounds().hi-plots[1].bounds().lo'),120);
 evaluate(`activatePlot(0);freezePlot();globalThis.cacheSnapshot=plots[0].view.frozen;globalThis.cacheSnapshotEnd=plots[0].bounds().hi;
  for(let i=40001;i<=55000;i++)add({kind:1,epoch:0,ms:i*10,values:[['retention.speed','rad/s',i]]});draw()`);
 assert(evaluate('plots[0].view.frozen===cacheSnapshot&&plots[0].bounds().hi===cacheSnapshotEnd&&plots[0].bounds().hi-plots[0].bounds().lo===100'),'Paused samples remain independent of overwritten live storage');
 evaluate(`$('window').value='5';$('window').onchange()`);
 assert(evaluate('plots[0].view.frozen===cacheSnapshot&&plots[0].bounds().hi===cacheSnapshotEnd&&plots[0].bounds().hi-plots[0].bounds().lo===5'),'Explicit duration shrink trims the paused snapshot without resuming');
 evaluate(`for(const p of plots){p.view.cacheSeconds=p.view.span=10}plots[0].resume();setPlotCount(1);clear();
  for(let i=0;i<2000;i++)add({kind:1,epoch:0,ms:i*10,values:[['retention.speed','rad/s',i]]});retainLiveCache(false,32);draw()`);
 assert.equal(evaluate("series.get('retention.speed|rad/s').points.length"),32);
 assert(evaluate("$('plotWindowState').textContent.includes('已达内存上限')"),'Memory pressure must be visible rather than silently shortening the requested history');
 evaluate(`$('window').value='.1';$('window').onchange()`);
 assert(evaluate("$('plotWindowState').dataset.limited==='false'"));
 evaluate(`clear();$('window').value='1';$('window').onchange();for(const ms of [0,10,500,990,1000,1500,1999,2000])add({kind:1,epoch:0,ms,values:[['retention.speed','rad/s',ms]]});draw()`);
 assert(evaluate('plots[0].groups()[0].items[0].s.points.length===4&&plots[0].bounds().lo===1&&plots[0].bounds().hi===2'),'Retention follows timestamps, including irregular sampling');
 evaluate(`add({kind:2,epoch:1,ms:10,values:[['retention.speed','rad/s',1]]});draw()`);
 assert(evaluate('plots[0].view.cacheSeconds===1&&plots[0].bounds().hi===.01&&cacheDropTime===-Infinity'),'Source reset preserves duration and clears old data and memory-limit state');
 evaluate(`for(const p of plots){p.view.cacheSeconds=p.view.span=10;p.selected.clear()}setPlotCount(1);clear()`);
 assert.equal(writes.length,before);
 console.log('PASS: duration cache: 100/120 s original samples, timestamp expiry, immediate shrink/gradual growth, shared and hidden canvases, independent pause copies, null gaps, explicit memory limit, source reset and zero commands');
}
async function main(){
 evaluate(code);await flush();assert.equal(writes.length,0,'Loading must not open a port or send a command');
 assert.equal(evaluate('HSO_REASONS[21]'),'EMF HIGH');
 assert(evaluate('plots.length===1&&plots[0].selected.size===0&&plots[0].layout.panels[0].items.length===0'),'Initial canvas is empty');
 evaluate('toggleDemo();demoFrame()');assert.equal(writes.length,0);assert(evaluate('demo && !connected && series.size===6'));
 assert(evaluate('plots[0].selected.size===0&&plots[0].layout.panels[0].items.length===0'),'Incoming demo data never binds itself');
 await assert.rejects(evaluate('connect()'),/演示/);evaluate('toggleDemo()');
 context.wireBytes=new Uint8Array(fs.readFileSync('build/telemetry-r86/wire-fixture.bin'));
 const kinds=evaluate(`(()=>{const p=new WireParser(),frames=[];for(let i=0;i<wireBytes.length;i+=7)frames.push(...p.feed(wireBytes.subarray(i,i+7)));for(const f of frames){if(f.kind===16)decodeStatus(f);else channels(f)}return Array.from(new Set(frames.map(f=>f.kind)))})()`);
 assert.equal(kinds.length,12);
 context.rpcBytes=new Uint8Array(rpcBytes);assert.equal(evaluate('new WireParser().feed(rpcBytes).length'),2);
 assert.equal(evaluate('decodeReply(new WireParser().feed(rpcBytes)[0]).settings.speed_x10'),123);
 const reference=reply(50,'Get');
 for(let i=0;i<reference.length;i++){const bad=Uint8Array.from(reference);bad[i]^=64;context.bad=bad;context.good=reference;assert.equal(evaluate('(()=>{const p=new WireParser();return p.feed(bad).length+p.feed(good).length})()'),1)}
 evaluate('parser=new WireParser();parser.feed(new Uint8Array(100000))');assert(evaluate('parser.buffer.length')<12);
 await evaluate('connect()');assert.equal(serialOptions.baudRate,115200);assert.deepEqual(writes,['Format:Native','@1:Get']);
 assert.equal(evaluate('boardConfig.firmware'),115);assert.equal(elements.get('motorModel').value,'2804');
 assert.equal(evaluate('outputFormat'),'Native');
 // Settings editor units, real row-frame layout, import validation and Stop cancellation.
 evaluate(`$('speedLimit').value=240;$('posSpeed').value=150;$('speedStep').value=1.7`);
 const startSettings=writes.length;await evaluate('applyEditorSettings()');
 const edited=writes.slice(startSettings).map(x=>x.replace(/^@\d+:/,''));
 assert(edited.includes('App:1,2,240')&&edited.includes('App:1,1,150')&&edited.includes('App:1,8,17'));
 assert.equal(edited[0],'App:1,2,240');assert.equal(elements.get('speedLimit').value,240,'ACK does not clobber unsent editor drafts');
 evaluate(`$('posSpeed').value=241`);await assert.rejects(evaluate('applyEditorSettings()'),/Pos Speed/);
 evaluate('populateSettings()');
 context.rowBytes=new Uint8Array(rowBase);
 assert.equal(evaluate('decodeRow(new WireParser().feed(rowBytes)[0]).value'),'24.0 V');
 assert.equal(evaluate('decodeRow(new WireParser().feed(rowBytes)[0]).axis'),2);
 await evaluate('readDiagnostics()');assert.equal(elements.get('diagRows').children.length,2);
 assert(evaluate("taskProgress.diagnostics.status==='done'&&taskProgress.diagnostics.completed===2&&taskProgress.diagnostics.total===2"),'Diagnostic progress uses the actual returned row count');
 await evaluate('clearDiagnostics()');assert(/:App:0,17,0$/.test(writes.at(-1)));
 evaluate('boardConfig.firmware=94');await assert.rejects(evaluate('clearDiagnostics()'),/R95/);evaluate('boardConfig.firmware=115');
 context.imported=JSON.parse(evaluate('JSON.stringify(editableConfig(boardConfig))'));
 evaluate(`imported.axes[1].speedLimit=300;imported.axes[1].posSpeed=180;imported.ledBrightness=63`);
 const commands=evaluate('importCommands(validateImport({format:"FOC-console-settings",version:1,editable:imported}))');
 assert(commands.includes('App:2,2,300')&&commands.includes('App:2,1,180')&&commands.includes('App:0,0,63'));
 assert.equal(commands[0],'App:0,14,0','Latch pause before any selector can schedule automatic alignment');
 assert(!commands.some(x=>/Speed:|Angle:|Torque:|Throttle:|Follow|Align:/.test(x)),'Import never starts or aligns a motor');
 assert.throws(()=>evaluate('validateImport({format:"FOC-console-settings",version:1,editable:{...imported,work:2,drive:1}})'),/Single M1/);
 assert.throws(()=>evaluate('checkedEditor(5,"IBUS",2.45)'),/0.1/);
 const failingStart=writes.length;replyResult=-1;await assert.rejects(evaluate(`runBatch(['App:1,2,200','App:1,1,100'])`));replyResult=1;
 assert.equal(writes.length-failingStart,1,'First rejection stops the import batch');
 autoReply=false;const batch=evaluate(`runBatch(['App:1,2,200','App:1,1,100'])`);batch.catch(()=>{});await flush();autoReply=true;const stopBefore=writes.length;
 await evaluate(`send('Stop')`);await assert.rejects(batch);assert.equal(writes.length,stopBefore+1,'Stop prevents subsequent batch commands');
 console.log('PASS: R88 editable Settings units/ranges/draft retention, actual C diagnostic rows, safe import validation/order/no motion, first-failure and Stop cancellation');
 await controlLinkage();
 context.jfBytes=new Uint8Array(fs.readFileSync('build/telemetry-r86/justfloat-fixture.bin'));
 assert(evaluate(`(()=>{const p=new JustFloatProbe();let detected=false;for(let i=0;i<jfBytes.length;i+=3)detected=p.feed(jfBytes.subarray(i,i+3))||detected;return detected})()`));
 assert(!evaluate(`new JustFloatProbe().feed(jfBytes.subarray(0,52))`),'One isolated frame cannot declare a format switch');
 const formatStart=writes.length;await evaluate("send('Format:JustFloat')");
 assert.deepEqual(writes.slice(formatStart),['Format:JustFloat']);assert(evaluate("outputFormat==='JustFloat'&&!formatVerified&&!boardConfig&&!pending.size"));
 await assert.rejects(evaluate("send('M1:Speed:20')"),/JustFloat/);
 await evaluate("send('Stop')");assert.equal(writes.at(-1),'Stop','Stop stays available without RPC in JustFloat');
 inject(context.jfBytes);await flush();assert(evaluate("outputFormat==='JustFloat'&&formatVerified&&!boardConfig"));
 await evaluate("send('Format:Native')");assert.equal(writes.at(-2),'Format:Native');assert(/:\s*Get$/.test(writes.at(-1)));
 assert(evaluate("outputFormat==='Native'&&boardConfig.firmware===115"));
 autoReply=false;const formatWait=evaluate("send('Get')");formatWait.catch(()=>{});await flush();
 const detectBefore=writes.length;inject(context.jfBytes);await flush();await assert.rejects(formatWait,/JustFloat/);
 assert(evaluate("outputFormat==='JustFloat'&&!boardConfig&&!pending.size"));assert.equal(writes.length,detectBefore,'Board-side JustFloat does not trigger an automatic mode override or repeat commands');
 autoReply=true;evaluate('parser.last=[123,0x234FFFF]');await evaluate("send('Format:Native')");
 assert(evaluate('boardConfig!==null'),'Native recovery after a JustFloat/MCU reset starts a fresh timestamp/CSV segment');
 console.log('PASS: Native-first raw handshake, R101 settings compatibility, explicit global JustFloat request, fragmented real C float frames, native query cancellation, no automatic override and raw Stop/recovery');
 plotRegressions();
 multiPlotRegressions();
 cacheRegressions();
 await evaluate(`send('Work:Independent')`);assert.equal(evaluate('boardConfig.work'),2);
 replyResult=-1;await assert.rejects(evaluate(`send('Work:bad')`),/板端拒绝/);replyResult=1;
 autoReply=false;const uncertain=evaluate(`send('Speed:20')`);uncertain.catch(()=>{});await flush();const before=writes.length;
 for(const [id,t]of [...timers])if(t.ms===2500){timers.delete(id);t.fn()}
 await assert.rejects(uncertain,/结果未知/);assert.equal(writes.length,before,'Timeout must never retry movement');
 const waiting=evaluate(`send('Get')`);waiting.catch(()=>{});await flush();autoReply=true;
 await evaluate(`send('Stop')`);await assert.rejects(waiting,/停止/);assert(writes.at(-1).endsWith(':Stop'));
 evaluate(`add({kind:1,ms:100,epoch:0,values:[['m1.feedback_speed','rad/s',2],['vbus','V',20],['m1.position','rad',null]]});draw()`);assert.equal(evaluate('series.size'),3);
 evaluate(`add({kind:4,ms:110,epoch:0,values:[['hso.relative_position','rad',0]]});draw()`);assert.equal(evaluate('series.size'),1);
 context.invalidCsv=new Blob(['bad']);context.invalidCsv.name='bad.csv';await assert.rejects(evaluate('loadCsv(invalidCsv)'),/CSV 格式/);
 assert(evaluate("taskProgress.csv.status==='error'&&taskProgress.csv.completed===3&&taskProgress.csv.total===3"),'A parse failure must not be reported as completed');
 const rows=['host_time,board_ms,sequence,epoch,packet_kind,field,unit,value'];for(let i=0;i<3000;i++)rows.push(`t,${i*10},${i},0,1,m1.feedback_speed,rad/s,${i}`);
 rows.push('t,30000,3000,0,16,status,json,"{}"','t,30000,3001,0,32,configuration,json,"{}"');
 context.csvFile=new Blob(['\ufeff'+rows.join('\r\n')+'\r\n']);context.csvFile.name='test.csv';evaluate("plots[0].control('window').value='100';plots[0].control('window').onchange()");await evaluate('loadCsv(csvFile)');
 assert(evaluate('replaying'));assert.equal(evaluate('[...series.values()][0].points.length'),3000);assert.equal(evaluate('latestTime'),29.99);
 assert(evaluate("taskProgress.csv.status==='done'&&taskProgress.csv.completed===csvFile.size&&taskProgress.csv.total===csvFile.size"),'CSV progress counts original UTF-8 bytes including the BOM');
 await assert.rejects(evaluate(`send('M1:Speed:10')`),/历史回放/);await evaluate(`send('Stop')`);
 evaluate('replaying=false;recording=true;recordBytes=16*1024*1024-1;recordChunks=[];recordFrame(new WireParser().feed(wireBytes)[0])');assert(!evaluate('recording'));assert(downloads.some(x=>x.endsWith('.csv')));
 evaluate('recordWriter={write:()=>{throw Error("must not enqueue")}};recording=true;recordQueuedBytes=1024*1024-1;recordFrame(new WireParser().feed(wireBytes)[0])');assert(!evaluate('recording'));assert(evaluate('recordQueuedBytes')<1024*1024);
 evaluate('recordWriter=null;recordQueuedBytes=0');
 boardReply[17]=115;evaluate('boardConfig.firmware=115');evaluate("$('calibrationAxis').value='1'");
 let startWrites=writes.length;await elements.get('applyDetection').onclick();assert(writes.at(-1).endsWith(':App:1,18,0'));
 await elements.get('restoreMotorPreset').onclick();assert(writes.at(-1).endsWith(':App:1,19,0'));
 assert(writes.slice(startWrites).every(x=>!/:Speed:|:Angle:|:Torque:|:Throttle:|HSO:Use/.test(x)));
 assert.equal(evaluate("formatMotorParams('M:1920/440/9290')"),'MEASURED · R 1.920 Ω · Ls 440 µH · Flux 9.290 mWb');
 assert(evaluate("formatMotorParams('P:1800/1300/0').startsWith('PRESET')"));
 console.log('PASS: R104 explicit per-axis calibration apply/preset commands, current parameter units, no motion command or automatic replay');
 boardReply.writeUInt32LE(boardReply.readUInt32LE(52)|(1<<28),52);await evaluate("send('Get')");
 assert.equal(evaluate('boardConfig.customMask'),1);
 choose('workMode','SingleM1');await evaluate('applyWorkMode()');choose('driveMode','Encoder');await evaluate('applyDriveMode()');
 assert(options('motorModel').includes('Custom1')&&!options('motorModel').includes('Custom2'));
 choose('motorModel','Custom1');await evaluate('applyMotorModel()');
 assert(writes.at(-1).endsWith(':Motor1:Custom1')&&evaluate('boardConfig.profiles[0]===3'));
 assert.equal(evaluate('confirmedModel(boardConfig,"M1")'),'Custom1');
 choose('workMode','SingleM2');await evaluate('applyWorkMode()');choose('motorModel','Custom1');await evaluate('applyMotorModel()');
 assert(writes.at(-1).endsWith(':Motor2:Custom1')&&evaluate('boardConfig.profiles[1]===3'));
 const customWrites=writes.length;evaluate("$('customSlot').value='2'");await elements.get('saveCustom').onclick();
 assert(writes.at(-1).endsWith(':App:0,20,2')&&writes.length===customWrites+1);
 const originalRow=Buffer.from(rowBase);rowBase.fill(0,20,76);rowBase.write('R/L/Flux',20);rowBase.write('M:1920/440/9290',44);
 await evaluate('readDiagnostics()');assert(elements.get('diagRows').children[0].children[1].textContent.includes('1.920 Ω'));
 originalRow.copy(rowBase);
 assert.throws(()=>evaluate('validateImport({format:"FOC-console-settings",version:1,editable:{...editableConfig(boardConfig),profiles:[4,0]}})'));
 console.log('PASS: R105 Custom availability, per-axis model readback and linked selectors, explicit slot save without motion, missing-slot import rejection');

 hangWrite=true;const hung=evaluate(`send('Get')`);hung.catch(()=>{});await flush();for(const [id,t]of [...timers])if(t.ms===2500){timers.delete(id);t.fn()}await assert.rejects(hung,/超时/);
 await evaluate('disconnect()');assert.equal(portClosed,1);assert(!evaluate('connected'));assert.equal(evaluate('boardConfig'),null);
 hangWrite=false;await evaluate('connect()');assert.equal(evaluate('parser.resets'),0);assert(evaluate('lastEpoch')>1);assert(writes.at(-1).endsWith(':Get'));await evaluate('disconnect()');assert.equal(portClosed,2);
 console.log('PASS: offline HTML; actual C wire/RPC bytes, CRC corruption/resync, Web Serial handshake, settings readback, IDs/rejection/timeout/no retry, priority Stop, hung-write disconnect, plots, bounded CSV replay and recording');
}
main().catch(e=>{console.error(e);process.exitCode=1});
