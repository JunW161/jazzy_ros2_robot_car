/* Logic checks in a minimal DOM harness; not a replacement for Chromium rendering. */
'use strict';
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const root=path.resolve(__dirname,'../delivery_robot_web/static');
const html=fs.readFileSync(path.join(root,'index.html'),'utf8');
const ids=[...html.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);
assert.equal(new Set(ids).size,ids.length,'HTML IDs must remain unique');
class Element{
  constructor(tag='div'){this.tagName=tag.toUpperCase();this.children=[];this.value='';this.disabled=false;this.hidden=false;this.className='';this.checked=true;this.textContent='';this.dataset={};this.classList={toggle(){},add(){},remove(){}};}
  append(...children){this.children.push(...children);}
  replaceChildren(...children){this.children=children;}
  get options(){return this.children;}
  querySelector(){return this.children[0]?.children?.[0]||null;}
}
const elements=Object.fromEntries(ids.map(id=>[id,new Element()]));
elements.missionTimeline.children=Array.from({length:4},()=>new Element('li'));
elements.deliveryMapMode.value='clean';
const calls=[],notices=[],errors=[];
let heldRequest=null,navCancelResponse=null,missionCancelResponse=null;
const context={
  console,Date,Number,Math,JSON,Set,Array,Promise,String,encodeURIComponent,
  $:id=>{assert.ok(elements[id],`Unknown UI element ${id}`);return elements[id];},
  document:{createElement:tag=>new Element(tag),querySelector:()=>({click(){}})},
  latest:{},loggedIn:false,socket:null,currentTab:'delivery',mapRows:[],
  WebSocket:{OPEN:1},
  MapCanvas:class{constructor(){this.draws=0;}draw(){this.draws++;}clear(){this.meta=null;}fit(){}async set(meta){this.meta=meta;}},
  setInterval(){},
  text(id,value){assert.ok(elements[id],id);elements[id].textContent=value;},
  showNotice(message,good){notices.push({message,good});},
  task:fn=>async event=>{try{return await fn(event);}catch(error){errors.push(error);}},
  refreshMaps:async()=>{},loadLocalizationMap:async()=>{},selectMap:async()=>{},
  api:async(url,method='GET',body)=>{
    calls.push({url,method,body});
    if(url==='/api/navigation/cancel'&&navCancelResponse)return navCancelResponse;
    if(url==='/api/missions/cancel'&&missionCancelResponse)return missionCancelResponse;
    if(heldRequest&&url==='/api/missions/start')return new Promise(resolve=>{heldRequest.resolve=resolve;});
    if(url.endsWith('/history'))return {items:[]};
    if(url.includes('/map-match/'))return {matches:true,message:'Map matches'};
    if(url.endsWith('/points'))return [];
    return {success:true,accepted:true,mission_id:'m1',message:'Accepted'};
  },
};
vm.createContext(context);
const run=source=>vm.runInContext(source,context);
run(fs.readFileSync(path.join(root,'delivery.js'),'utf8'));
assert.equal(calls.filter(c=>c.method==='POST').length,0,'Page load must never start robot');
assert.ok(elements.startNavigation.disabled);
assert.ok(elements.startMission.disabled);
assert.ok(elements.sendSingleGoal.disabled);
context.loggedIn=true;context.socket={readyState:1};
context.latest={mode:'ROS',motion:{armed:false},processes:{navigation:{running:true,managed:true,external:false},localization:{context:{map_id:'map1'}}},
 tf_map:{x:1,y:2,yaw:0,age:.1},navigation:{ready:true,active_map_id:'map1',issues:[],lifecycle:{},
 mux:{available:true,mode:'STOP'},mission:{available:true,receive_age:.1,state_name:'IDLE',active:false},goal:{state:'IDLE'},plan:{frame_id:'map',points:[[0,0],[1,1]],age:.1}}};
run("deliveryLastStatus=Date.now();deliveryMapId='map1';deliveryMapCheck={map_id:'map1',matches:true,at:Date.now()};");
elements.pickupPoint.value='p1';elements.dropoffPoint.value='p2';elements.singleGoalPoint.value='p1';
run('renderDelivery()');
assert.equal(elements.startMission.disabled,false,'Ready idle mission should accept explicit start');
assert.equal(elements.sendSingleGoal.disabled,false);
assert.ok(elements.startNavigation.disabled,'Do not duplicate ready navigation');
assert.equal(elements.stopNavigation.disabled,false,'Owned navigation can stop');
context.latest.processes.navigation.external=true;
run('renderDelivery()');assert.ok(elements.stopNavigation.disabled,'External navigation cannot be killed from Web');
context.latest.processes.navigation.external=false;
run("deliveryMapCheck.matches=false;renderDelivery()");
assert.ok(elements.startMission.disabled,'Unverified map blocks mission');
assert.ok(elements.sendSingleGoal.disabled,'Unverified map blocks goal');
assert.equal(run('deliveryPose()'),null,'Do not plot TF on unverified map');
run("deliveryMapCheck.matches=true;");
context.latest.navigation.mission={available:true,receive_age:.1,state_name:'WAITING_PICKUP',active:true,mission_id:'m1',pickup_name:'Kho',dropoff_name:'Bàn A'};
run('renderDelivery()');
assert.equal(elements.confirmPickup.hidden,false);assert.equal(elements.confirmPickup.disabled,false);
assert.ok(elements.confirmDropoff.hidden);assert.ok(elements.startMission.disabled);assert.ok(elements.deliveryMap.disabled);
context.latest.navigation.mission.receive_age=5;
run('renderDelivery()');assert.ok(elements.confirmPickup.disabled,'Stale mission cannot advance');
context.latest.navigation.mission.receive_age=.1;
run('deliveryDisconnected()');assert.ok(elements.confirmPickup.disabled);assert.ok(elements.startMission.disabled);
assert.equal(elements.globalStop.disabled,false,'STOP remains available after disconnect');
run('deliveryStatusReceived()');
context.latest.navigation.mission={available:true,receive_age:.1,state_name:'CANCELED',active:false,mission_id:'m1'};
run('renderDelivery()');
assert.equal(elements.missionStatusBadge.textContent,'ĐÃ HỦY');
assert.ok(!elements.missionStatusBadge.className.includes('success'),'Cancel is not completion');
context.latest.navigation.goal={state:'ACTIVE',distance_remaining:.5};
run('renderDelivery()');assert.ok(elements.startMission.disabled);assert.equal(elements.cancelSingleGoal.disabled,false);
assert.equal(run('currentPath().length'),2);
context.latest.navigation.plan.frame_id='odom';assert.equal(run('currentPath().length'),0,'No wrong-frame path overlay');
context.latest.navigation.plan.frame_id='map';context.latest.mode='OFFLINE';
run('renderDelivery()');assert.ok(elements.startMission.disabled);assert.ok(elements.startNavigation.disabled);assert.equal(run('currentPath().length'),0);
context.latest.mode='ROS';context.latest.navigation.goal={state:'IDLE'};context.latest.navigation.mission={available:true,receive_age:.1,state_name:'IDLE',active:false};
run('renderDelivery()');
(async()=>{
  heldRequest={};
  const event={preventDefault(){}};
  const first=elements.missionForm.onsubmit(event);
  await Promise.resolve();
  await elements.missionForm.onsubmit(event);
  const starts=calls.filter(c=>c.url==='/api/missions/start');
  assert.equal(starts.length,1,'Double click must not create concurrent starts');
  assert.equal(JSON.stringify(starts[0].body),JSON.stringify({map_id:'map1',pickup_id:'p1',dropoff_id:'p2'}));
  heldRequest.resolve({accepted:true,mission_id:'m2'});await first;
  navCancelResponse={success:false,cancel_requested:true,confirmed:false,message:'Đang chờ Nav2 xác nhận hủy.'};
  await elements.cancelSingleGoal.onclick();
  assert.equal(notices.at(-1).good,false,'A pending cancellation must not look successful');
  assert.match(notices.at(-1).message,/Đang chờ/);
  navCancelResponse={success:true,cancel_requested:true,confirmed:false};
  await elements.cancelSingleGoal.onclick();
  assert.equal(notices.at(-1).good,false,'Receipt alone does not confirm goal termination');
  navCancelResponse={success:true,cancel_requested:true,confirmed:true};
  await elements.cancelSingleGoal.onclick();
  assert.equal(notices.at(-1).good,true,'Explicit confirmed termination can show acknowledgment');
  missionCancelResponse={success:true,cancel_confirmed:false,message:'Đang chờ hủy nhiệm vụ.'};
  await elements.cancelMission.onclick();
  assert.equal(notices.at(-1).good,false,'Mux STOP alone is not mission cancellation');
  missionCancelResponse={success:true,cancel_confirmed:true};
  await elements.cancelMission.onclick();
  assert.equal(notices.at(-1).good,true);

  // Exercise the real transport and STOP acknowledgment policy without starting a browser or ROS.
  const appSource=fs.readFileSync(path.join(root,'app.js'),'utf8');
  let responseBody={},responseOk=true;
  const transportNotices=[];
  const transport=vm.createContext({JSON,Error,csrf:'',armKey:null,held:[0,0],
    invalidateMotion(){transport.armKey=null;transport.held=[0,0];},
    showNotice:(message,good)=>transportNotices.push({message,good}),
    fetch:async()=>({ok:responseOk,json:async()=>responseBody})});
  vm.runInContext(appSource.slice(appSource.indexOf('async function api('),appSource.indexOf('\nfunction task(')),transport);
  vm.runInContext(appSource.slice(appSource.indexOf('async function stopMotion('),appSource.indexOf("\n$('globalStop')")),transport);
  responseBody={success:false,cancel_requested:true,confirmed:false,message:'Đang chờ kết quả Nav2.'};
  assert.equal((await vm.runInContext("api('/api/navigation/cancel','POST',{})",transport)).confirmed,false,
    'Pending cancellation is a pending result, not an HTTP failure');
  responseBody={success:false,message:'Từ chối thao tác.'};
  await assert.rejects(vm.runInContext("api('/api/missions/start','POST',{})",transport),/Từ chối/);
  responseBody={ok:true,mux_confirmed:true,cancel_confirmed:false,message:'Mux đã STOP.'};
  await vm.runInContext('stopMotion()',transport);
  assert.equal(transportNotices.at(-1).good,false,'STOP must show incomplete cancellation');
  responseBody={ok:true,mux_confirmed:true};
  await vm.runInContext('stopMotion()',transport);
  assert.equal(transportNotices.at(-1).good,false,'Missing acknowledgment is not confirmation');
  responseBody={ok:true,mux_confirmed:true,cancel_confirmed:true,message:'Đã xác nhận STOP.'};
  await vm.runInContext('stopMotion()',transport);
  assert.equal(transportNotices.at(-1).good,true);
  responseOk=false;responseBody={detail:'Mất kết nối.'};
  await vm.runInContext('stopMotion()',transport);
  assert.equal(transportNotices.at(-1).good,undefined);
  assert.match(transportNotices.at(-1).message,/Chưa xác nhận dừng/);
  assert.equal(errors.length,0,errors.map(e=>String(e)).join('\n'));
  console.log('UI LOGIC: PASS (map identity, offline/stale gating, ownership, cancellation/STOP acknowledgments, path frame, explicit start, duplicate click)');
})();
