/* Execute both production scripts in a deterministic DOM harness.
 * No ROS, HTTP server, motor publisher, or browser is started by these checks.
 * Actual layout verification is a separate Playwright smoke test.
 */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const root = path.resolve(__dirname, '../delivery_robot_web/static');
const html = fs.readFileSync(path.join(root, 'index.html'), 'utf8');
const idList = [...html.matchAll(/\bid="([^"]+)"/g)].map(match => match[1]);
assert.equal(idList.length, new Set(idList).size, 'HTML IDs must be unique');
assert.doesNotMatch(html, /Tạo bản đồ (?:&|và) lái xe/);

class Element {
  constructor(tag = 'div', attributes = '') {
    this.tagName = tag.toUpperCase(); this.children = []; this.value = '';
    this.disabled = /\bdisabled\b/.test(attributes); this.hidden = /\bhidden\b/.test(attributes);
    this.checked = /\bchecked\b/.test(attributes); this.textContent = ''; this.dataset = {};
    this.className = (attributes.match(/\bclass="([^"]+)"/) || [])[1] || '';
    this.id = (attributes.match(/\bid="([^"]+)"/) || [])[1] || '';
    this.value = (attributes.match(/\bvalue="([^"]*)"/) || [])[1] || '';
    for (const match of attributes.matchAll(/data-([a-z-]+)="([^"]*)"/g)) this.dataset[match[1]] = match[2];
    const self = this;
    this.classList = {
      contains(token) { return self.className.split(/\s+/).includes(token); },
      toggle(token, force) {
        const set = new Set(self.className.split(/\s+/).filter(Boolean));
        const on = force === undefined ? !set.has(token) : force;
        if (on) set.add(token); else set.delete(token);
        self.className = [...set].join(' '); return on;
      },
      add(...tokens) { for (const token of tokens) this.toggle(token, true); },
      remove(...tokens) { for (const token of tokens) this.toggle(token, false); },
    };
    this.parentElement = {classList: {toggle(){}}}; this.listeners = {};
    this.width = 640; this.height = 420;
  }
  append(...children) { this.children.push(...children); }
  replaceChildren(...children) { this.children = children; }
  addEventListener(name, callback) { this.listeners[name] = callback; }
  click() { if (!this.disabled) return this.onclick?.({preventDefault(){}, target:this}); }
  querySelector() { return this.children[0]?.children[0] || null; }
  get options() { return this.children; }
  getBoundingClientRect() { return {width:640, height:420, left:0, top:0}; }
  getContext() { return new Proxy({}, {get: () => () => {}}); }
  setPointerCapture() {}
}
const all = [...html.matchAll(/<([a-z][a-z0-9]*)\b([^>]*)>/gi)].map(match => new Element(match[1], match[2]));
const elements = Object.fromEntries(all.filter(item => item.id).map(item => [item.id, item]));
const navs = all.filter(item => item.dataset.tab);
assert.equal(navs[0].dataset.tab, 'overview', 'System must be the first navigation item');
assert.ok(navs[0].classList.contains('active'));
assert.equal(elements['tab-overview'].hidden, false);
assert.equal(elements['tab-delivery'].hidden, true);
elements.missionTimeline.children = Array.from({length:4}, () => new Element('li'));
elements.straightRpm.value = '40'; elements.turnRpm.value = '20';
elements.deliveryMapMode.value = 'clean'; elements.localizationMapMode.value = 'clean';
const timers = [], requests = [];
let responder = async (url) => url === '/api/session' ? {ok:false, body:{detail:'No session'}} : {ok:true, body:{}};
const find = selector => {
  const tab = selector.match(/data-tab="([^"]+)"/);
  if (tab) return navs.filter(item => item.dataset.tab === tab[1]);
  if (selector === '[data-tab]') return navs;
  if (selector.startsWith('.')) return all.filter(item => item.classList.contains(selector.slice(1)));
  return [];
};
const context = {
  console, Date, Number, Math, JSON, Set, Array, Promise, String, Error, encodeURIComponent,
  document:{getElementById:id => {assert.ok(elements[id], `Unknown element ${id}`); return elements[id];},
    createElement:tag => new Element(tag), querySelector:selector => find(selector)[0] || null,
    querySelectorAll:find, addEventListener(){}, hidden:false, title:''},
  window:{addEventListener(){}}, location:{protocol:'http:',host:'ui.test'},
  WebSocket:class {static OPEN=1;}, ResizeObserver:class {observe(){}},
  Image:class {}, setInterval(fn,delay){timers.push({fn,delay});}, setTimeout(){},
  confirm:()=>true, prompt:()=>null,
  fetch:async (url, options={}) => {
    requests.push({url, method:options.method||'GET', body:options.body ? JSON.parse(options.body) : undefined});
    const result=await responder(url, options);
    return {ok:result.ok!==false, json:async()=>result.body||{}};
  },
};
vm.createContext(context);
const run=source=>vm.runInContext(source, context);
run(fs.readFileSync(path.join(root,'app.js'),'utf8'));
run(fs.readFileSync(path.join(root,'delivery.js'),'utf8'));
assert.equal(run('currentTab'),'overview');
assert.equal(requests.filter(r=>r.method==='POST').length,0,'Page initialization must not start components or motor control');

function fresh(state={}) {
  const base={mode:'ROS',ros_ok:true,processes:{},system_operation:{state:'idle'},navigation_operation:{state:'idle'},
    nodes:[],motion:{armed:false},motion_ready:true,motion_issues:[],mapping_issues:[],
    odom_publishers:1,scan_publishers:1,odom:{hz:30,receive_age:.03,stamp_age:.04},scan:{hz:6,receive_age:.07,stamp_age:.08},
    motor:{safety_age:.05,wheel_age:.05,state_name:'IDLE',interlock:0,rpm_l:0,rpm_r:0},
    tf_odom:{age:.03},navigation:{ready:false,mux:{available:true,mode:'STOP'},mission:{available:true,receive_age:.1,state_name:'IDLE',active:false}}};
  run(`latest=${JSON.stringify({...base,...state})};loggedIn=true;socket={readyState:1};lastStatusAt=Date.now();deliveryLastStatus=Date.now();renderStatus();`);
}
function processRow(name) {
  return elements.processCards.children.find(row=>row.children[0].children[0].textContent===name);
}
function action(name,index) { return processRow(name).children[2].children[index]; }

(async()=>{
  await Promise.resolve(); // Initial unauthenticated GET is allowed to settle.
  fresh();
  assert.equal(elements.odomHz.textContent,'30.0 Hz');
  assert.match(elements.odomAge.textContent,/1 nguồn · Nhận: 30 ms · Dấu thời gian: 40 ms/);
  assert.equal(run('formatAge(null)'),'—'); assert.equal(run('formatAge(Infinity)'),'—');
  assert.equal(run('sampleDisplay(null, 0).value'),'—');
  assert.match(run('sampleDisplay({receive_age:.03,stamp_age:.04,hz:30},null).detail'),/^— nguồn/);
  assert.equal(run('sampleDisplay({receive_age:.03,stamp_age:.04,hz:30},null).value'),'—');
  assert.equal(run('sampleDisplay({receive_age:.01,stamp_age:1e9,hz:30},1).value'),'30.0 Hz');
  assert.match(run('sampleDisplay({receive_age:.01,stamp_age:1e9,hz:30},1).detail'),/Lệch dấu thời gian/);
  assert.doesNotMatch(run('sampleDisplay({receive_age:1e9,stamp_age:1e9,hz:30},0).detail'),/ngày|100000/);
  assert.equal(run('sampleDisplay({receive_age:10,stamp_age:10,hz:30},1).value'),'Mất dữ liệu');
  assert.equal(run('sampleDisplay({receive_age:.01,stamp_age:.01,hz:30},1,false).value'),'Gián đoạn');

  // Recoverable external process stop stays callable; unrecognized ownership does not.
  fresh({processes:{lidar:{running:true,external:true,recoverable:true},localization:{running:true,external:true,recoverable:true},
    navigation:{running:true,managed:true},costmap:{running:true,external:false}},nodes:['/amcl']});
  assert.equal(action('LiDAR',0).disabled,true);
  assert.equal(action('LiDAR',1).disabled,false,'Recognized external LiDAR must be stoppable');
  run('latest.processes.lidar.recoverable=false;renderStatus()');
  assert.equal(action('LiDAR',1).disabled,true,'Unknown external processes must not be stopped');
  run('latest.processes.lidar.recoverable=true;latest.ros_ok=false;latest.ros_error="ROS callback failed";currentTab="localization";renderStatus()');
  assert.equal(elements.rosExecutorError.hidden,false);
  assert.equal(elements.odomHz.textContent,'Gián đoạn');
  assert.equal(elements.disconnectSystem.disabled,false,'Cleanup must remain available on a ROS fault');
  assert.equal(elements.connectSystem.disabled,false,'Fresh HTTP status must allow explicit backend executor recovery');
  assert.equal(action('LiDAR',1).disabled,false);
  assert.equal(elements.stopLocalization.disabled,false,'Active navigation must not prevent cascading localization cleanup');
  assert.equal(elements.stopCostmap.disabled,false,'Managed Costmap cleanup remains available');
  assert.equal(elements.globalStop.disabled,false);
  assert.equal(elements.startNavigation.disabled,true,'ROS failure must not expose a start action');
  assert.equal(elements.navReadyLabel.classList.contains('success'),false);

  // Age checks use receipt monotonic ages and hide stale rendering instead of trusting cached rates.
  fresh({processes:{agent:{running:true,external:false}},ros_error:'',ros_executor:{state:'running',last_error:'Past executor error'}});
  assert.equal(elements.rosExecutorError.hidden,true,'Past error text is not a current executor failure');
  run('lastStatusAt=Date.now()-5000;deliveryLastStatus=Date.now()-5000;renderStatus()');
  assert.equal(elements.odomHz.textContent,'Gián đoạn');
  assert.equal(elements.armButton.disabled,true);
  assert.equal(elements.disconnectSystem.disabled,false);
  assert.equal(elements.globalStop.disabled,false);

  // Async operations remain pending until the status stream reports a result.
  fresh();
  const connectOp={id:'connect-1',kind:'connect',state:'running',message:'Đang bật hệ thống',steps:[{component:'agent',state:'running'}]};
  responder=async url=>({body:{operation:url==='/api/system/disconnect'?{id:'stop-1',kind:'disconnect',state:'running',message:'Đang dừng hệ thống',steps:[]}:connectOp}});
  const before=requests.length;
  await elements.connectSystem.onclick();
  assert.deepEqual(requests.slice(before).map(r=>r.url),['/api/system/connect'],'Connect must use a single async endpoint');
  assert.equal(elements.connectSystem.disabled,true);
  assert.equal(elements.disconnectSystem.disabled,false,'Stop must interrupt a pending connect');
  assert.equal(elements.systemOperation.hidden,false);
  assert.equal(elements.systemOperationSteps.children.length,1);
  assert.equal(elements.notice.classList.contains('good'),false,'Acceptance is not completion');
  const beforeStop=requests.length;
  await elements.disconnectSystem.onclick();
  assert.deepEqual(requests.slice(beforeStop).map(r=>r.url),['/api/system/disconnect'],'Cleanup must not wait for a preliminary STOP RPC');
  assert.equal(elements.disconnectSystem.disabled,true,'Repeated pending disconnect is disabled');
  run('latest.system_operation={id:"stop-1",kind:"disconnect",state:"succeeded",message:"Đã dừng tiến trình.",warnings:["Chưa xác nhận STOP từ mux."],steps:[]};renderStatus()');
  assert.equal(elements.systemOperationState.textContent,'Có lưu ý');
  assert.equal(elements.systemOperationState.classList.contains('success'),false);
  assert.match(elements.systemOperationMessage.textContent,/Chưa xác nhận STOP/);
  assert.equal(elements.notice.classList.contains('good'),false,'Host cleanup is not a physical STOP acknowledgment');
  assert.equal(elements.connectSystem.disabled,false,'Successful cleanup permits restart');

  // Actual Costmap buttons preserve generic start/stop endpoint behavior.
  fresh({processes:{localization:{running:true,managed:true},costmap:{running:false}},nodes:['/amcl'],tf_map:{x:0,y:0,yaw:0,age:.05}});
  responder=async url=>({body:{operation:{id:url,kind:url.endsWith('/start')?'component_start':'component_stop',component:'costmap',state:'running',steps:[]}}});
  let routeCount=requests.length;
  await elements.startCostmap.onclick();
  assert.equal(requests[routeCount].url,'/api/process/costmap/start');
  run('latest.system_operation.state="succeeded";latest.processes.costmap.running=true;renderStatus()');
  routeCount=requests.length;
  await elements.stopCostmap.onclick();
  assert.equal(requests[routeCount].url,'/api/process/costmap/stop');
  run('latest.system_operation.state="succeeded";renderStatus()');
  routeCount=requests.length;
  await elements.openRviz.onclick();
  assert.equal(requests[routeCount].url,'/api/rviz/start');

  // A late arm response cannot reacquire the UI lease after the user presses STOP.
  fresh();
  let resolveArm;
  responder=async url=>url==='/api/control/arm'?await new Promise(resolve=>{resolveArm=resolve;}):
    {body:{ok:true,mux_confirmed:true,cancel_confirmed:true,message:'Đã xác nhận STOP.'}};
  const pendingArm=elements.armButton.onclick();
  await Promise.resolve();
  assert.equal(run('armPending'),true);
  await elements.globalStop.onclick();
  resolveArm({body:{key:'obsolete-lease'}});
  await pendingArm;
  assert.equal(run('armKey'),null,'Late arm key must be discarded after STOP');
  assert.equal(run('armPending'),false);
  assert.ok(all.filter(e=>e.classList.contains('drive')).every(e=>e.disabled));
  assert.equal(requests.filter(r=>r.url==='/api/control/command').length,0);

  // Offline and crashed states must never be visually marked ready for delivery.
  fresh({mode:'OFFLINE',ros_ok:false});
  assert.equal(elements.connectSystem.disabled,true);assert.equal(elements.startMission.disabled,true);
  assert.equal(elements.globalStop.disabled,false);
  fresh({ros_ok:false,navigation:{ready:true,mux:{available:true,mode:'STOP'},lifecycle:{planner_server:'active'},mission:{available:true,receive_age:.1,state_name:'IDLE'}}});
  assert.equal(elements.navigationBadge.classList.contains('success'),false);
  assert.equal(elements.navLifecycle.children[0].classList.contains('active'),false);
  assert.match(elements.navigationReadiness.textContent,/gián đoạn/);
  console.log('UI SYSTEM: PASS (both scripts, system first, IDs, fresh data, fault cleanup, dependent stops, async warnings, restart, Costmap/RViz routes, late arm cancellation)');
})().catch(error=>{console.error(error);process.exitCode=1;});
