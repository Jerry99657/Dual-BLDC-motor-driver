"""Exercise real Edge Streams and the HTML transport under injected faults.

The fake driver retains the granted SerialPort across a device-lost error,
like the reported Windows case. No physical serial port or motor is used.
"""
import json
import os
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'build/serial-recovery'
OUT.mkdir(parents=True, exist_ok=True)
FAKE = r"""(() => {
 const b=window.fakeBoard={writes:[],sequence:1000,ms:5000,opens:0,closes:0,
  opened:false,hold:false,openFailure:false,closeFailure:false,hangWrite:false,renewFailures:0};
 let readStream,writeStream,rx,tx,renew=false,fatal=false,pendingWrite=null;
 function newRead(){
  readStream=new ReadableStream({start(c){rx=c}});
  if(b.renewFailures>0){b.renewFailures--;renew=true;rx.error(new DOMException('buffer overrun','BufferOverrunError'))}
 }
 function reply(text){
  const m=text.match(/^@(\d+):Get$/);if(!m||b.hold)return;
  const bytes=Uint8Array.from(b.reply),v=new DataView(bytes.buffer);
  v.setUint16(4,b.sequence++ & 65535,true);v.setUint16(12,Number(m[1]),true);
  v.setUint32(8,b.ms+=10,true);v.setUint16(bytes.length-2,crc16(bytes.subarray(0,-2)),true);rx.enqueue(bytes);
 }
 const port={
  getInfo:()=>({usbVendorId:0x0483,usbProductId:0x5740}),
  open:async options=>{
   b.options=options;if(b.openFailure)throw new DOMException('Open failed','NetworkError');
   if(b.opened)throw new DOMException('Already open','InvalidStateError');
   b.opened=true;b.opens++;fatal=false;renew=false;newRead();
   writeStream=new WritableStream({start(c){tx=c},write(bytes){
    const text=new TextDecoder().decode(bytes).trim();b.writes.push(text);
    if(b.hangWrite)return new Promise((resolve,reject)=>{pendingWrite={resolve,reject}});
    reply(text);
   }});
  },
  close:async()=>{
   b.closes++;if(b.closeFailure)throw new DOMException('Driver close failed','NetworkError');
   if(readStream?.locked||writeStream?.locked)throw Error('Streams still locked');
   if(pendingWrite){pendingWrite.reject(new DOMException('Port closed','AbortError'));pendingWrite=null}
   b.opened=false;try{rx.close()}catch{};
  },
  get readable(){if(!b.opened||fatal)return null;if(renew){renew=false;newRead()}return readStream},
  get writable(){return b.opened?writeStream:null}
 };
 b.port=port;
 b.push=bytes=>rx.enqueue(Uint8Array.from(bytes));
 b.readError=(name,message,terminal=false)=>{fatal=terminal;renew=!terminal;rx.error(new DOMException(message,name))};
 b.writeError=()=>tx.error(new DOMException('The device has been lost.','NetworkError'));
 b.finishWrite=()=>{pendingWrite.resolve();pendingWrite=null};
 b.endRead=()=>rx.close();
 const serial=new EventTarget();serial.requestPort=async()=>port;
 b.osDisconnect=()=>{const event=new Event('disconnect');Object.defineProperty(event,'port',{value:port});serial.dispatchEvent(event)};
 Object.defineProperty(navigator,'serial',{value:serial,configurable:true});
})();"""

with sync_playwright() as playwright:
    browser = playwright.chromium.launch(channel='msedge' if os.name == 'nt' else 'chromium', headless=True)
    try:
        page = browser.new_page(viewport={'width': 1440, 'height': 1000})
        page_errors = []
        page.on('pageerror', lambda error: page_errors.append(str(error)))
        page.add_init_script(FAKE)
        page.goto((ROOT / 'pc_tool/index.html').as_uri())
        page.evaluate('data=>{fakeBoard.reply=data}', list((ROOT / 'build/console-r87/rpc-fixture.bin').read_bytes()[:106]))

        def ready():
            page.wait_for_function("connected&&boardConfig?.firmware===132&&!connecting&&!closing")

        def closed():
            page.wait_for_function('!connected&&!closing&&!connecting&&port===null&&reader===null&&writer===null')

        def connect():
            page.evaluate('connect()')
            ready()

        # Native handshake and no-data idle do not falsely classify a disconnect.
        connect()
        assert page.evaluate('fakeBoard.options.bufferSize') == 65536
        page.wait_for_timeout(100)
        assert page.evaluate('connected')

        # An overflow replaces only the read stream. Pending results become
        # unknown; partial Native bytes cannot leak into the new stream.
        page.evaluate("""()=>{
         fakeBoard.hold=true;window.waitingGet=send('Get').then(()=> 'unexpected',e=>e.message);
        }""")
        page.wait_for_function('pending.size===1')
        writes_before = page.evaluate('fakeBoard.writes.length')
        page.evaluate("fakeBoard.push(fakeBoard.reply.slice(0,23));fakeBoard.readError('BufferOverrunError','buffer overrun')")
        page.wait_for_function('serialSession?.recovering&&pending.size===0')
        assert '未知' in page.evaluate('waitingGet')
        assert page.evaluate('connected&&boardConfig===null&&parser.buffer.length===0')
        assert page.evaluate('fakeBoard.writes.length') == writes_before
        page.evaluate("fakeBoard.hold=false;fakeBoard.push(fakeBoard.reply)")
        page.wait_for_function('serialStats.recoveries===1')
        page.evaluate("send('Get')")
        ready()
        assert page.evaluate('serialStats.connections') == 1

        # An error cancels not-yet-submitted writes too. A queued command
        # cannot execute after its pending RPC has already been rejected.
        page.evaluate("""()=>{
         fakeBoard.hold=true;fakeBoard.hangWrite=true;
         window.inFlightGet=send('Get').then(()=> 'unexpected',e=>e.message);
         window.queuedGet=transmit('@60000:Get').then(()=> 'unexpected',e=>e.message);
        }""")
        page.wait_for_function('pending.size===1')
        page.evaluate("fakeBoard.readError('BufferOverrunError','overrun during write')")
        page.wait_for_function('serialSession?.recovering&&pending.size===0')
        page.evaluate('fakeBoard.hold=false;fakeBoard.hangWrite=false;fakeBoard.finishWrite();fakeBoard.push(fakeBoard.reply)')
        assert '取消' in page.evaluate('queuedGet')
        assert '未知' in page.evaluate('inFlightGet')
        assert '@60000:Get' not in page.evaluate('fakeBoard.writes')
        page.evaluate("send('Get')")
        ready()

        # The reported device-lost error: close both streams and the handle,
        # then reconnect the SAME granted object without a separate Disconnect.
        page.evaluate("fakeBoard.osDisconnect();fakeBoard.readError('NetworkError','The device has been lost.',true)")
        closed()
        assert 'NetworkError' in page.locator('#error').inner_text()
        assert '连接已释放' in page.locator('#error').inner_text()
        assert page.evaluate('serialStats.osDisconnects') == 1
        assert page.evaluate("serialStats.events.some(e=>e.type==='read-error'&&!e.recoverable&&e.name==='NetworkError')")
        connect()

        # A write-side failure is terminal even if the reader would otherwise
        # wait forever. Stream end, processing errors and handshake failure
        # use the same deterministic cleanup, with distinct diagnostics.
        page.evaluate('fakeBoard.writeError()')
        closed()
        assert page.evaluate('serialStats.writeErrors>=1')
        connect()
        page.evaluate('fakeBoard.endRead()')
        closed()
        connect()
        page.evaluate("window.savedConsume=consume;consume=()=>{throw Error('simulated render failure')};fakeBoard.push(fakeBoard.reply)")
        closed()
        assert page.evaluate("serialStats.events.some(e=>e.type==='processing-error'&&e.message==='simulated render failure')")
        page.evaluate('()=>{consume=savedConsume}')

        page.evaluate('fakeBoard.openFailure=true')
        assert 'Open failed' in page.evaluate("connect().then(()=>'',e=>e.message)")
        closed()
        page.evaluate('fakeBoard.openFailure=false;fakeBoard.hold=true')
        page.evaluate("()=>{window.handshakeResult=connect().then(()=> 'unexpected',e=>e.message)}")
        page.wait_for_function('pending.size===1')
        closed()
        assert '超时' in page.evaluate('handshakeResult')
        page.evaluate('fakeBoard.hold=false')
        connect()

        # Retry budget: streams that error before delivering bytes must stop,
        # rather than spin on an already-errored reader forever.
        page.evaluate("fakeBoard.renewFailures=20;fakeBoard.readError('BufferOverrunError','repeated overrun')")
        closed()
        page.evaluate('fakeBoard.renewFailures=0')
        connect()

        # A truly pending Streams write makes writer.abort wait. Disconnect
        # must release locks and call port.close BEFORE awaiting that promise.
        page.evaluate("()=>{fakeBoard.hangWrite=true;window.hungGet=send('Get').then(()=>'',e=>e.message)}")
        page.wait_for_function('pending.size===1')
        page.evaluate('disconnect()')
        closed()
        assert '未知' in page.evaluate('hungGet')
        page.evaluate('fakeBoard.hangWrite=false')
        connect()

        # Simultaneous terminal read and manual disconnect coalesce one close.
        before = page.evaluate('fakeBoard.closes')
        page.evaluate("fakeBoard.readError('NetworkError','The device has been lost.',true);disconnect()")
        closed()
        assert page.evaluate('fakeBoard.closes') == before + 1
        connect()

        # Failed close retains the handle for an explicit cleanup retry.
        page.evaluate('fakeBoard.closeFailure=true')
        assert 'Driver close failed' in page.evaluate("disconnect().then(()=>'',e=>e.message)")
        assert page.evaluate('port===fakeBoard.port&&!connected&&!closing&&reader===null&&writer===null')
        page.evaluate('fakeBoard.closeFailure=false;disconnect()')
        closed()
        connect()

        # Repeated successful recovery stays bounded and never sends motion.
        for _ in range(35):
            count = page.evaluate('serialStats.recoveries')
            page.evaluate("fakeBoard.readError('ParityError','simulated UART parity error')")
            page.wait_for_function('serialSession?.recovering')
            page.evaluate('fakeBoard.push(fakeBoard.reply)')
            page.wait_for_function('count=>serialStats.recoveries>count', arg=count)
            page.evaluate("send('Get')")
            ready()

        snapshot = page.evaluate('runtimeSnapshot()')
        assert snapshot['transport']['connections'] >= 10
        assert snapshot['transport']['recoveries'] == 37
        assert snapshot['transport']['firmware'] == 132
        assert snapshot['transport']['bytes'] > 0
        assert len(snapshot['transport']['events']) == 64
        assert snapshot['transport']['info'] == {'usbVendorId': 0x0483, 'usbProductId': 0x5740}
        assert snapshot['transport']['lastByteAgeMs'] is not None
        writes = page.evaluate('fakeBoard.writes')
        assert all(text == 'Format:Native' or text.endswith(':Get') for text in writes), writes
        page.evaluate('disconnect()')
        closed()
        assert not page_errors, page_errors
        (OUT / 'browser-report.json').write_text(json.dumps({'browser': browser.version,
            'source': 'fake Windows-style SerialPort and real Streams; no device',
            'writes': writes, 'diagnostics': snapshot, 'pageErrors': page_errors}, ensure_ascii=False, indent=2), encoding='utf-8')
        print('PASS: real Edge Streams: nonfatal read recovery, unknown pending results/no replay, device-lost same-port reconnect, writer failure, EOF, processing-error classification, open/handshake failure cleanup, retry limit, hung-write teardown, concurrent disconnect, failed-close retry, 35 repeated recoveries, bounded diagnostics; zero motor commands')
    finally:
        browser.close()
