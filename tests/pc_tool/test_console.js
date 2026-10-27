// Actual single-file script + model Web Serial/DOM. No browser or hardware.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync('pc_tool/index.html','utf8');
assert(!/<script[^>]+src=|<link[^>]+href=|fetch\(/i.test(html),'No external runtime dependency');
const code=html.match(/<script>([\s\S]*?)<\/script>/)[1],elements=new Map(),writes=[],timers=new Map(),downloads=[];
let timerId=0,serialOptions=null,portClosed=0,serialReader;
class Element{
 constructor(id=''){this.id=id;this.value=({window:'10',scrub:'1000',axis:'M1',motorAxis:'M1',action:'Speed'})[id]||'';this.textContent='';this.children=[];this.files=[];this.style={}}
 append(...items){this.children.push(...items)}replaceChildren(...items){this.children=items}
 getBoundingClientRect(){return {width:1000,height:480}}
 getContext(){return new Proxy({},{get:(o,k)=>o[k]||(()=>{}),set:(o,k,v)=>(o[k]=v,true)})}
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
function reply(id,command){const frame=Buffer.from(base);frame.writeUInt16LE(sequence++&65535,4);frame.writeUInt32LE(boardMs+=10,8);frame.writeUInt16LE(id,12);frame.writeInt8(replyResult,14);
 if(command==='Work:Independent')frame.writeUInt32LE((frame.readUInt32LE(52)&~7)|2,52);
 frame.writeUInt16LE(crc(frame.subarray(0,-2)),frame.length-2);return new Uint8Array(frame)}
function rowReply(id,command){const frame=Buffer.from(rowBase),m=command.match(/^GetRow:(\d+),(\d+),(\d+)$/);frame.writeUInt16LE(sequence++&65535,4);frame.writeUInt32LE(boardMs+=10,8);frame.writeUInt16LE(id,12);frame[16]=Number(m[1]);frame[17]=Number(m[2]);frame[18]=Number(m[3]);frame.writeUInt16LE(crc(frame.subarray(0,-2)),frame.length-2);return new Uint8Array(frame)}
function inject(bytes){serialReader.push(bytes)}
const fakePort={open:async options=>{serialOptions=options},close:async()=>{portClosed++},
 writable:{getWriter:()=>({write:async bytes=>{const text=new TextDecoder().decode(bytes).trim();writes.push(text);if(hangWrite)return new Promise((_,reject)=>{rejectWrite=reject});const m=text.match(/^@(\d+):(.*)$/);if(autoReply&&m)inject((m[2].startsWith('GetRow:')?rowReply:reply)(Number(m[1]),m[2]))},abort:async()=>{rejectWrite?.(Error('aborted'));rejectWrite=null},releaseLock(){}})},
 readable:{getReader:()=>{let waiting=null,queue=[];serialReader={read:()=>queue.length?Promise.resolve(queue.shift()):new Promise(resolve=>{waiting=resolve}),push:bytes=>{if(waiting){const f=waiting;waiting=null;f({value:bytes,done:false})}else queue.push({value:bytes,done:false})},cancel:async()=>{if(waiting){const f=waiting;waiting=null;f({done:true})}},releaseLock(){}};return serialReader}}};
async function main(){
 evaluate(code);await flush();assert.equal(writes.length,0,'Loading must not open a port or send a command');
 evaluate('toggleDemo();demoFrame()');assert.equal(writes.length,0);assert(evaluate('demo && !connected && series.size===6'));await assert.rejects(evaluate('connect()'),/演示/);evaluate('toggleDemo()');
 context.wireBytes=new Uint8Array(fs.readFileSync('build/telemetry-r86/wire-fixture.bin'));
 const kinds=evaluate(`(()=>{const p=new WireParser(),frames=[];for(let i=0;i<wireBytes.length;i+=7)frames.push(...p.feed(wireBytes.subarray(i,i+7)));for(const f of frames){if(f.kind===16)decodeStatus(f);else channels(f)}return Array.from(new Set(frames.map(f=>f.kind)))})()`);
 assert.equal(kinds.length,12);
 context.rpcBytes=new Uint8Array(rpcBytes);assert.equal(evaluate('new WireParser().feed(rpcBytes).length'),2);
 assert.equal(evaluate('decodeReply(new WireParser().feed(rpcBytes)[0]).settings.speed_x10'),123);
 const reference=reply(50,'Get');
 for(let i=0;i<reference.length;i++){const bad=Uint8Array.from(reference);bad[i]^=64;context.bad=bad;context.good=reference;assert.equal(evaluate('(()=>{const p=new WireParser();return p.feed(bad).length+p.feed(good).length})()'),1)}
 evaluate('parser=new WireParser();parser.feed(new Uint8Array(100000))');assert(evaluate('parser.buffer.length')<12);
 await evaluate('connect()');assert.equal(serialOptions.baudRate,115200);assert.deepEqual(writes,['@1:Get']);
 assert.equal(evaluate('boardConfig.firmware'),91);assert.equal(elements.get('motorModel').value,'2804');
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
 await evaluate(`send('Work:Independent')`);assert.equal(evaluate('boardConfig.work'),2);
 replyResult=-1;await assert.rejects(evaluate(`send('Work:bad')`),/板端拒绝/);replyResult=1;
 autoReply=false;const uncertain=evaluate(`send('Speed:20')`);uncertain.catch(()=>{});await flush();const before=writes.length;
 for(const [id,t]of [...timers])if(t.ms===2500){timers.delete(id);t.fn()}
 await assert.rejects(uncertain,/结果未知/);assert.equal(writes.length,before,'Timeout must never retry movement');
 const waiting=evaluate(`send('Get')`);waiting.catch(()=>{});await flush();autoReply=true;
 await evaluate(`send('Stop')`);await assert.rejects(waiting,/停止/);assert(writes.at(-1).endsWith(':Stop'));
 evaluate(`add({kind:1,ms:100,epoch:0,values:[['m1.feedback_speed','rad/s',2],['vbus','V',20],['m1.position','rad',null]]});draw()`);assert.equal(evaluate('series.size'),3);
 evaluate(`add({kind:4,ms:110,epoch:0,values:[['hso.relative_position','rad',0]]});draw()`);assert.equal(evaluate('series.size'),1);
 const rows=['host_time,board_ms,sequence,epoch,packet_kind,field,unit,value'];for(let i=0;i<3000;i++)rows.push(`t,${i*10},${i},0,1,m1.feedback_speed,rad/s,${i}`);
 rows.push('t,30000,3000,0,16,status,json,"{}"','t,30000,3001,0,32,configuration,json,"{}"');
 context.csvFile=new Blob(['\ufeff'+rows.join('\r\n')+'\r\n']);context.csvFile.name='test.csv';await evaluate('loadCsv(csvFile)');
 assert(evaluate('replaying'));assert(evaluate('[...series.values()][0].points.length<=2200'));assert.equal(evaluate('latestTime'),29.99);
 await assert.rejects(evaluate(`send('M1:Speed:10')`),/历史回放/);await evaluate(`send('Stop')`);
 evaluate('replaying=false;recording=true;recordBytes=16*1024*1024-1;recordChunks=[];recordFrame(new WireParser().feed(wireBytes)[0])');assert(!evaluate('recording'));assert(downloads.some(x=>x.endsWith('.csv')));
 evaluate('recordWriter={write:()=>{throw Error("must not enqueue")}};recording=true;recordQueuedBytes=1024*1024-1;recordFrame(new WireParser().feed(wireBytes)[0])');assert(!evaluate('recording'));assert(evaluate('recordQueuedBytes')<1024*1024);
 evaluate('recordWriter=null;recordQueuedBytes=0');
 hangWrite=true;const hung=evaluate(`send('Get')`);hung.catch(()=>{});await flush();for(const [id,t]of [...timers])if(t.ms===2500){timers.delete(id);t.fn()}await assert.rejects(hung,/超时/);
 await evaluate('disconnect()');assert.equal(portClosed,1);assert(!evaluate('connected'));assert.equal(evaluate('boardConfig'),null);
 hangWrite=false;await evaluate('connect()');assert.equal(evaluate('parser.resets'),0);assert(evaluate('lastEpoch')>1);assert(writes.at(-1).endsWith(':Get'));await evaluate('disconnect()');assert.equal(portClosed,2);
 console.log('PASS: offline HTML; actual C wire/RPC bytes, CRC corruption/resync, Web Serial handshake, settings readback, IDs/rejection/timeout/no retry, priority Stop, hung-write disconnect, plots, bounded CSV replay and recording');
}
main().catch(e=>{console.error(e);process.exitCode=1});
