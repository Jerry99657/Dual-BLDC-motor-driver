// Independent analytic oracles against the exact engine embedded in the HTML.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync('pc_tool/index.html','utf8');
const engine=html.split('/* MEASUREMENT_ENGINE_BEGIN:')[1].split('/* MEASUREMENT_ENGINE_END */')[0];
const code=engine.slice(engine.indexOf('*/')+2),scope=vm.createContext({});vm.runInContext(code,scope);
const analyze=(r,y,o={})=>scope.responseAnalyze(r,y,{start:0,end:5,maxGap:.1,mode:'tracking',bandPct:2,absBand:0,hold:.2,frequency:4,unit:'rad',...o});
function samples(fn,end=5,dt=.01){const n=Math.round(end/dt)+1;return {t:Float64Array.from({length:n},(_,i)=>i*dt),v:Float64Array.from({length:n},(_,i)=>fn(i*dt))}}
function near(a,b,epsilon=1e-8){assert(Number.isFinite(a)&&Math.abs(a-b)<epsilon,`${a} expected ${b} within ${epsilon}`)}
async function main(){
 let r=samples(t=>2*t),y=samples(t=>2*t+.25),a=await analyze(r,y);
 near(a.tracking.rms,.25);near(a.tracking.bias,.25);near(a.tracking.peak,.25);near(a.tracking.stddev,0);near(a.tracking.iae,1.25);near(a.quality.coverage,1);
 const irregular={t:Float64Array.of(0,.1,.3,1),v:Float64Array.of(0,0,0,0)},offset={t:irregular.t,v:Float64Array.of(1,1,1,1)};
 a=await analyze(irregular,offset,{end:1,maxGap:1});near(a.tracking.rms,1);near(a.tracking.iae,1);near(a.quality.maxInterval,.7);
 for(const sign of [1,-1]){
  r=samples(t=>t<1?3:3+sign*10);y=samples(t=>t<1?3:3+sign*10*(1-Math.exp(-(t-1)/.2)));
  a=await analyze(r,y,{mode:'step'});assert(a.step.valid);
  near(a.step.onset,1);near(a.step.rise10to90,.2*Math.log(9),.001);near(a.step.delay10,-.2*Math.log(.9),.001);
  near(a.step.reach90,.2*Math.log(10),.001);near(a.step.settlingTime,.2*Math.log(50),.011);near(a.step.overshootPct,0);
 }
 r=samples(t=>t<1?0:10);y=samples(t=>t<1?0:10+Math.max(0,1-Math.abs(t-2))*1.7);
 a=await analyze(r,y,{mode:'step'});near(a.step.overshootPct,17);assert(a.step.settlingTime>1.8);
 y=samples(t=>t<1?0:10*(1-Math.exp(-(t-1)/.2)));
 a=await analyze(r,y,{mode:'step',end:1.1});assert(a.step.rise10to90===null&&a.step.settlingTime===null);
 a=await analyze(samples(t=>t),samples(t=>.9*t),{mode:'step'});assert(!a.step.valid);
 a=await analyze(samples(t=>t<1?0:t<2?10:20),samples(t=>t<1?0:10),{mode:'step'});assert(!a.step.valid);
 const missing=samples(t=>t>1&&t<2?NaN:0);
 a=await analyze(samples(()=>0),missing);assert(a.quality.coverage<.81&&a.quality.coverage>.79);near(a.tracking.rms,0);
 a=await analyze(r,missing,{mode:'step'});assert(!a.step.valid);
 const partial=samples(t=>t*2+.25,4,.02);a=await analyze(samples(t=>2*t),partial);near(a.tracking.bias,.25);assert(a.quality.coverage<=.8+1e-8);
 await assert.rejects(analyze({t:Float64Array.of(0,1,.5),v:Float64Array.of(0,1,2)},samples(()=>0)),/递增/);
 for(const lag of [.03,-.02]){
  r=samples(t=>10000000+.17*Math.sin(2*Math.PI*4*t));y=samples(t=>5000000+.17*.75*Math.sin(2*Math.PI*4*(t-lag)));
  a=await analyze(r,y,{mode:'sine'});assert(a.sine.valid,JSON.stringify(a.sine));near(a.sine.gain,.75,1e-6);near(a.sine.equivalentLagSeconds,lag,1e-6);near(a.sine.r2[0],1,1e-8);
 }
 a=await analyze(samples(()=>1),samples(()=>1),{mode:'sine'});assert(!a.sine.valid);
 a=await analyze(samples(t=>Math.sin(2*Math.PI*4*t)),samples(t=>Math.sin(2*Math.PI*4*t)),{mode:'sine',frequency:7});assert(!a.sine.valid);
 a=await analyze(samples(t=>Math.sin(2*Math.PI*4*t),5,.05),samples(t=>Math.sin(2*Math.PI*4*t),5,.05),{mode:'sine'});assert(!a.sine.valid);
 a=await analyze(samples(t=>Math.sin(2*Math.PI*4*t),5,.025),samples(t=>Math.sin(2*Math.PI*4*t),5,.025),{mode:'sine'});assert(a.sine.valid);
 // Wire group.error = slave - target: reconstruct target without encoder origins.
 const master=t=>.17*Math.sin(2*Math.PI*4*t),slave=t=>.17*.8*Math.sin(2*Math.PI*4*(t-.04));
 a=await analyze(samples(t=>slave(t)-master(t)),samples(slave),{derive:'follow',mode:'sine'});
 assert(a.sine.valid);near(a.sine.gain,.8,1e-8);near(a.sine.equivalentLagSeconds,.04,1e-8);
 let checkpoints=0;
 // The actual optional checkpoint can stop a long job before publication.
 await assert.rejects(scope.responseAnalyze(samples(()=>0),samples(()=>0),{start:0,end:5,maxGap:.1,mode:'tracking',bandPct:2,absBand:0,hold:.2},async()=>{if(++checkpoints===2)throw Error('cancelled')}),/cancelled/);
 console.log('PASS: analytic positive/negative step delay/rise/settling, overshoot, censored response, ramps/multiple steps, weighted errors, NaN/gaps, interpolation/no extrapolation, time rejection, DC-offset sine fits, lag/lead, sampling gates, Follow wire sign and cancellation');
}
main().catch(e=>{console.error(e);process.exitCode=1});
