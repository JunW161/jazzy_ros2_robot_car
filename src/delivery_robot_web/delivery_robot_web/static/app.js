'use strict';
const $ = id => document.getElementById(id);
let csrf = '', socket = null, currentTab = 'overview', loggedIn = false;
let latest = {}, mapRows = [], selectedMap = null, points = [], editingPoint = null;
let selectedLocalizationMapId = '', localizationPoints = [], initialPoseMode = false, initialPoseDraft = null;
let lastArmAt = 0, systemBusy = false, lastStatusAt = 0;
let watchedSystemOperation = null, reportedSystemOperation = null;
let armKey = null, sequence = 0, held = [0,0], commandPending = false, liveRevision = -1;
let motionGeneration = 0, armPending = false;
function invalidateMotion(){motionGeneration++;armKey=null;held=[0,0];}
let liveFetch = false, pointMode = false;
let liveMapMode = 'clean', localizationMapMode = 'clean', localizationImageKey = '';
let localizationMapRefreshBusy = false, lastLocalizationMapCheck = 0;
const titles = {overview:'Hệ thống',delivery:'Giao hàng',mapping:'Tạo bản đồ',library:'Bản đồ & điểm',localization:'Định vị',devices:'Thiết bị',logs:'Nhật ký'};
function showNotice(text, good=false) { $('notice').textContent=text; $('notice').classList.toggle('good',good); $('notice').hidden=false; }
async function api(path, method='GET', body) {
  const response=await fetch(path,{method,credentials:'same-origin',headers:{'Content-Type':'application/json','X-CSRF-Token':csrf},body:body===undefined?undefined:JSON.stringify(body)});
  const data=await response.json().catch(()=>({detail:'Không đọc được phản hồi.'}));
  if(!response.ok) throw new Error(typeof data.detail==='string'?data.detail:JSON.stringify(data.detail));
  const cancelPending=path==='/api/navigation/cancel'&&data.cancel_requested===true&&data.confirmed===false;
  if(method!=='GET'&&!cancelPending&&(data.success===false||data.accepted===false||data.ok===false)) throw new Error(data.message||data.note||'Hệ thống chưa xác nhận thao tác.');
  return data;
}
function task(fn){return async event=>{try{await fn(event);}catch(err){showNotice(err.message);}};}
function text(id, value){$(id).textContent=value;}
function button(label,fn,style='ghost'){const b=document.createElement('button'); b.textContent=label;b.className=style;b.addEventListener('click',task(fn));return b;}
async function enter(session){csrf=session.csrf;loggedIn=true;$('loginView').hidden=true;$('workspace').hidden=false;$('password').value='';connect();await loadSettings();await refreshMaps();if(typeof enterDelivery==='function')await enterDelivery();}
$('loginForm').addEventListener('submit',async e=>{e.preventDefault();$('loginError').textContent='';try{await enter(await api('/api/login','POST',{username:$('username').value,password:$('password').value}));}catch(err){text('loginError',err.message);}});
$('logoutButton').onclick=task(async()=>{await stopMotion();await api('/api/logout','POST',{});loggedIn=false;socket?.close();$('workspace').hidden=true;$('loginView').hidden=false;csrf='';});
function connect(){
  if(!loggedIn)return;
  socket=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/ws/status`);
  socket.onopen=()=>text('streamState','Đang kết nối');
  socket.onmessage=e=>{try{latest=JSON.parse(e.data);lastStatusAt=Date.now();if(typeof deliveryStatusReceived==='function')deliveryStatusReceived();text('streamState',latest.ros_ok===false?'ROS gián đoạn':'Đã kết nối');renderStatus();}catch(err){showNotice('Không đọc được trạng thái hệ thống: '+err.message);}};
  socket.onerror=()=>text('streamState','Lỗi kết nối');
  socket.onclose=()=>{invalidateMotion();lastStatusAt=0;text('streamState','Mất kết nối');renderStatus();if(typeof deliveryDisconnected==='function')deliveryDisconnected();if(loggedIn)setTimeout(connect,2000);};
}
function statusIsFresh(){return lastStatusAt>0&&Date.now()-lastStatusAt<3000&&socket?.readyState===WebSocket.OPEN;}
function busyOperation(){return latest.system_operation?.state==='running'||latest.navigation_operation?.state==='running';}
function formatAge(value){
  if(typeof value!=='number'||!Number.isFinite(value))return '—';
  const absolute=Math.abs(value),sign=value<0?'−':'';
  if(absolute<1)return `${sign}${Math.round(absolute*1000)} ms`;
  if(absolute<60)return `${sign}${absolute.toFixed(1)} s`;
  if(absolute<3600)return `${sign}${(absolute/60).toFixed(1)} phút`;
  if(absolute<86400)return `${sign}${(absolute/3600).toFixed(1)} giờ`;
  return `${sign}${(absolute/86400).toFixed(1)} ngày`;
}
function sampleDisplay(sample={},publishers=null,available=true){
  sample=sample&&typeof sample==='object'?sample:{};
  const sources=Number.isFinite(publishers)&&publishers>=0?publishers:null;
  const receive=sample.receive_age,stamp=sample.stamp_age;
  const hasReceive=typeof receive==='number'&&Number.isFinite(receive)&&receive>=0;
  const fresh=available&&sources>0&&hasReceive&&receive<1.5;
  const hz=typeof sample.hz==='number'&&Number.isFinite(sample.hz)?sample.hz:null;
  const value=!available?'Gián đoạn':!hasReceive||sources===null||sources===0?'—':!fresh?'Mất dữ liệu':hz===null?'—':`${hz.toFixed(1)} Hz`;
  const received=available&&sources>0&&hasReceive?formatAge(receive):'—';
  const timestamp=available&&sources>0&&typeof stamp==='number'&&Number.isFinite(stamp)?
    (hasReceive&&receive<1.5&&Math.abs(stamp)>60?'Lệch dấu thời gian':formatAge(stamp)):'—';
  return {value,detail:`${sources===null?'—':sources} nguồn · Nhận: ${received} · Dấu thời gian: ${timestamp}`,fresh};
}
function renderSystemOperation(){
  const op=latest.system_operation||{},panel=$('systemOperation');
  panel.hidden=!op.id;
  if(!op.id)return;
  const labels={connect:'Kết nối hệ thống',disconnect:'Dừng hệ thống',component_start:'Khởi động thành phần',component_stop:'Dừng thành phần'};
  const states={running:'Đang xử lý',succeeded:'Hoàn tất',failed:'Có lỗi'};
  text('systemOperationTitle',labels[op.kind]||'Thao tác hệ thống');
  const warnings=Array.isArray(op.warnings)?op.warnings:[];
  text('systemOperationState',op.state==='succeeded'&&warnings.length?'Có lưu ý':states[op.state]||op.state||'—');
  $('systemOperationState').className='badge '+(op.state==='succeeded'&&!warnings.length?'success':op.state==='failed'?'danger':warnings.length?'warn':'neutral');
  text('systemOperationMessage',[op.message,...warnings].filter(Boolean).join('\n'));
  const steps=$('systemOperationSteps');steps.replaceChildren();
  for(const step of op.steps||[]){
    const row=document.createElement('div');row.className='operation-step '+(step.state||'pending');
    const name=document.createElement('strong');name.textContent=processNames[step.component]||step.component;
    const state=document.createElement('span');state.textContent=({pending:'Chờ',running:'Đang xử lý',succeeded:'Xong',failed:'Lỗi',external:'Ngoài phiên',skipped:'Không cần'})[step.state]||step.state;
    row.append(name,state);if(step.message)row.title=step.message;steps.append(row);
  }
  if(watchedSystemOperation===op.id&&op.state!=='running'&&reportedSystemOperation!==op.id){
    reportedSystemOperation=op.id;showNotice([op.message,...warnings].filter(Boolean).join(' ')||states[op.state]||'Đã nhận kết quả.',op.state==='succeeded'&&!warnings.length);
  }
}
function acceptOperation(result,fallback){
  if(result.operation){latest.system_operation=result.operation;watchedSystemOperation=result.operation.id;renderStatus();showNotice(result.operation.message||fallback);return;}
  const rows=result.results||[];
  showNotice(result.message||rows.map(row=>`${processNames[row.component]||row.component}: ${row.message||''}`).join(' · ')||fallback,
    result.ok!==false&&rows.every(row=>row.ok!==false));
}
function renderStatus(){
  const s=latest,fresh=statusIsFresh(),rosAvailable=fresh&&s.ros_ok!==false,busy=busyOperation();
  if(!rosAvailable&&(armKey||armPending))invalidateMotion();
  text('modeLabel',s.mode==='OFFLINE'?'Chế độ ngoại tuyến':'ROS 2 Jazzy');
  const odom=sampleDisplay(s.odom,s.odom_publishers,rosAvailable),scan=sampleDisplay(s.scan,s.scan_publishers,rosAvailable);
  text('odomHz',odom.value);text('scanHz',scan.value);text('odomAge',odom.detail);text('scanAge',scan.detail);
  $('odomHz').classList.toggle('metric-stale',!odom.fresh);$('scanHz').classList.toggle('metric-stale',!scan.fresh);
  const tfAge=s.tf_odom?.age,tfValid=rosAvailable&&Number.isFinite(tfAge)&&tfAge>=-.75&&tfAge<1;
  text('tfState',tfValid?'Đang cập nhật':!rosAvailable?'Gián đoạn':s.tf_odom?'Dữ liệu cũ':'Chưa có');
  text('tfAge',`Dữ liệu TF: ${rosAvailable?formatAge(tfAge):'—'}`);
  const slamLabel={active:'Đang tạo',inactive:'Chưa kích hoạt',unconfigured:'Chưa cấu hình',unknown:'Chưa chạy',offline:'Chưa chạy'};
  text('slamState',rosAvailable?(slamLabel[s.slam_state]||'Chưa chạy'):'—');text('mapState',rosAvailable&&s.map_revision?'Đã nhận bản đồ':'Chưa có bản đồ');
  const op=s.system_operation||{},running=Object.values(s.processes||{}).filter(p=>p.running).length;
  text('systemBadge',s.mode==='OFFLINE'?'Ngoại tuyến':op.state==='running'?(op.kind==='disconnect'?'Đang dừng':'Đang xử lý'):s.ros_ok===false?'ROS gián đoạn':!fresh?'Chưa kết nối':running?'Đang hoạt động':'Đã dừng');
  $('systemBadge').className='status-pill '+(s.ros_ok===false?'error':running&&fresh?'connected':'');
  text('connectionHint',op.state==='running'?(op.message||'Đang cập nhật trạng thái…'):s.ros_ok===false?'Kết nối ROS cần được khôi phục.':running?`${running} thành phần đang chạy.`:'Kết nối hệ thống để bắt đầu vận hành.');
  const rosError=s.ros_error||(s.ros_ok===false?s.ros_executor?.last_error:'');
  $('rosExecutorError').hidden=s.mode==='OFFLINE'||(!rosError&&s.ros_ok!==false);
  text('rosExecutorError',rosError||'Kết nối ROS bị gián đoạn. Các thao tác dừng hệ thống vẫn khả dụng.');
  text('motionState',s.motion?.reason||'Chưa nhận quyền điều khiển.');
  if(!s.motion?.armed&&armKey&&Date.now()-lastArmAt>500){armKey=null;held=[0,0];}
  $('armButton').disabled=armPending||!!armKey||!s.motion_ready||!rosAvailable||busy;
  document.querySelectorAll('.drive').forEach(b=>b.disabled=!armKey||!rosAvailable);
  const checks=$('readiness');checks.replaceChildren();
  const issues=s.mapping_issues||[];
  for(const line of rosAvailable?(issues.length?issues:['Sẵn sàng tạo bản đồ.']):['Chưa có dữ liệu ROS mới.']){
    const item=document.createElement('div');item.className='checkline'+(!issues.length&&rosAvailable?' ok':'');item.textContent=line;checks.append(item);
  }
  $('startSlam').disabled=!rosAvailable||busy||!!issues.length||s.nodes?.includes('/slam_toolbox');
  $('activateSlam').disabled=!rosAvailable||busy||s.slam_state!=='inactive'||!s.processes?.slam?.running;
  $('stopSlam').disabled=!s.processes?.slam?.running;
  $('saveMap').disabled=!rosAvailable||busy||!s.map_revision||s.slam_state!=='active'||!!s.motion?.armed;
  const m=s.motor||{},n=v=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(1):'—';
  const wheelFresh=rosAvailable&&Number.isFinite(m.wheel_age)&&m.wheel_age<1.5;
  const safeFresh=rosAvailable&&Number.isFinite(m.safety_age)&&m.safety_age<1.5;
  text('motorStatus',`Motor: ${safeFresh?m.state_name||'—':'—'} · Trái ${wheelFresh?n(m.rpm_l):'—'} RPM · Phải ${wheelFresh?n(m.rpm_r):'—'} RPM · Khóa ${safeFresh?m.interlock??'—':'—'}`);
  const mi=$('motionIssues');mi.replaceChildren();
  for(const reason of s.motion_issues||[]){const item=document.createElement('div');item.className='checkline';item.textContent=reason;mi.append(item);}
  const mode=s.navigation?.mux?.mode;
  text('robotAuthority',({STOP:'Dừng',MANUAL:'Điều khiển tay',NAVIGATION:'Tự hành'})[mode]||'Chưa kết nối');
  text('activeMapName',s.navigation?.active_map_name||s.processes?.localization?.context?.map_name||'Chưa chọn');
  text('navReadyLabel',s.navigation?.ready&&rosAvailable?'Sẵn sàng':'Chưa sẵn sàng');
  $('navReadyLabel').className='badge '+(s.navigation?.ready&&rosAvailable?'success':'neutral');
  const u=s.usb_job||{};
  text('wifiState',u.message||({idle:'Chưa có tác vụ',built:'Đã build',building:'Đang build',flashing:'Đang nạp',failed:'Có lỗi'})[u.state]||u.state||'Chưa có tác vụ');
  $('buildWifi').disabled=!!u.busy||s.mode==='OFFLINE'||busy;
  $('flashWifi').disabled=u.state!=='built'||!!u.busy||s.mode==='OFFLINE'||busy;
  $('connectSystem').disabled=s.mode==='OFFLINE'||!!u.busy||systemBusy||busy||!fresh;
  $('disconnectSystem').disabled=s.mode==='OFFLINE'||!!u.busy||op.state==='running'&&op.kind==='disconnect';
  $('straightRpm').disabled=!!armKey;$('turnRpm').disabled=!!armKey;
  text('driveTelemetry',`Yêu cầu L/R: ${wheelFresh?n(m.target_l):'—'} / ${wheelFresh?n(m.target_r):'—'} RPM · Đo L/R: ${wheelFresh?n(m.rpm_l):'—'} / ${wheelFresh?n(m.rpm_r):'—'} RPM`);
  syncRpmOptions();updateSpeedLabel();renderProcesses();renderSystemOperation();
  if(currentTab==='mapping'){updateLive();liveView.draw();}
  if(currentTab==='localization'){renderLocalizationStatus();localizationView?.draw();}
  if(typeof renderDelivery==='function')renderDelivery();
}
const processNames={agent:'micro-ROS Agent',odom_bridge:'Trạng thái bánh xe',ekf_odom:'Ước lượng vị trí',robot_model:'Mô hình robot',lidar:'LiDAR',cmd_mux:'Quyền điều khiển',localization:'Định vị',navigation:'Navigation',mission:'Nhiệm vụ giao hàng',slam:'Tạo bản đồ',costmap:'Costmap',rviz:'RViz'};
const processDescriptions={agent:'Bộ điều khiển ESP32',odom_bridge:'Odometry & khớp bánh xe',ekf_odom:'Encoder & IMU',robot_model:'URDF & hệ tọa độ',lidar:'Dữ liệu cảm biến',cmd_mux:'Lái tay / tự hành',localization:'AMCL & bản đồ',navigation:'Lập đường & điều khiển',mission:'Lấy hàng & giao hàng',slam:'SLAM Toolbox',costmap:'Kiểm tra môi trường',rviz:'Hiển thị kỹ thuật'};
let processSignature='';
function processPresentation(process={}){
  if(process.status==='stopping')return {label:'Đang dừng',tone:'progress'};
  if(process.status==='failed'||process.status==='degraded')return {label:'Có lỗi',tone:'bad'};
  if(process.status==='starting')return {label:'Đang khởi động',tone:'progress'};
  if(process.context?.provided_by)return {label:'Tích hợp',tone:process.running?'good':'idle'};
  if(process.external)return {label:process.recoverable?'Đã nhận diện':'Ngoài phiên',tone:'external'};
  if(process.running)return {label:'Đang chạy',tone:'good'};
  return {label:'Đã dừng',tone:'idle'};
}
function renderProcesses(){
  const op=latest.system_operation||{},fresh=statusIsFresh(),busy=busyOperation();
  const sig=JSON.stringify([latest.processes,latest.mode,latest.usb_job?.busy,latest.ros_ok,fresh,op.id,op.state,op.component,busy]);
  if(sig===processSignature)return;processSignature=sig;
  const root=$('processCards');root.replaceChildren();let running=0;
  for(const [key,label] of Object.entries(processNames)){
    const process=latest.processes?.[key]||{},external=!!process.external,integrated=process.context?.provided_by;
    if(process.running)running++;
    const card=document.createElement('div');card.className='process-card component-row';
    const name=document.createElement('div');name.className='component-name';
    const h=document.createElement('h4');h.textContent=label;const sub=document.createElement('span');sub.textContent=processDescriptions[key];name.append(h,sub);card.append(name);
    const status=document.createElement('div');status.className='component-status';
    const presentation=processPresentation(process),pill=document.createElement('span');pill.className='component-pill '+presentation.tone;pill.textContent=presentation.label;status.append(pill);
    if(integrated){const note=document.createElement('small');note.textContent=`Cùng ${processNames[integrated]||integrated}`;status.append(note);}
    if(process.message){status.title=process.message;if(presentation.tone==='bad'){const error=document.createElement('small');error.className='component-error';error.textContent=process.message;status.append(error);}}
    card.append(status);
    const row=document.createElement('div');row.className='button-row component-actions';
    const start=button(key==='localization'?'Mở định vị':'Bật',()=>startProcess(key),'ghost');
    start.disabled=latest.mode==='OFFLINE'||!!latest.usb_job?.busy||busy||!fresh||latest.ros_ok===false||!!process.running||!!integrated;
    const stop=button('Dừng',()=>stopProcess(key),'ghost');
    const starting=op.state==='running'&&op.kind==='component_start'&&op.component===key;
    stop.disabled=latest.mode==='OFFLINE'||!!latest.usb_job?.busy||(!process.running&&!starting)||(external&&!process.recoverable)||!!integrated||process.status==='stopping';
    const log=button('Nhật ký',async()=>{document.querySelector('[data-tab="logs"]').click();$('logSelect').value=key;text('processLog',(await api(`/api/process/${key}/log`)).text);});
    row.append(start,stop,log);card.append(row);root.append(card);
  }
  text('componentCount',`${running}/${Object.keys(processNames).length} đang chạy`);
}
async function startProcess(name){
  if(busyOperation())return;
  if(name==='localization'){document.querySelector('[data-tab="localization"]').click();return;}
  await stopMotion(false);showNotice(`Đang bật ${processNames[name]||name}…`);
  const result=await api(name==='rviz'?'/api/rviz/start':`/api/process/${name}/start`,'POST',{});
  if(name==='navigation'&&result.operation){latest.navigation_operation=result.operation;showNotice(result.operation.message||'Đang bật Navigation.');return;}
  acceptOperation(result,`Đang bật ${processNames[name]||name}…`);
}
async function stopProcess(name){
  invalidateMotion();
  const path=name==='localization'?'/api/localization/stop':name==='rviz'?'/api/rviz/stop':`/api/process/${name}/stop`;
  const result=await api(path,'POST',{});
  if(name==='navigation'&&result.operation){latest.navigation_operation=result.operation;showNotice(result.operation.message||'Đang dừng Navigation.');return;}
  acceptOperation(result,`Đang dừng ${processNames[name]||name}…`);
}
$('overviewLocalization').onclick=()=>document.querySelector('[data-tab="localization"]').click();
$('startSlam').onclick=task(()=>startProcess('slam'));$('stopSlam').onclick=task(()=>stopProcess('slam'));
$('activateSlam').onclick=task(async()=>{await api('/api/slam/activate','POST',{});showNotice('Đã gửi yêu cầu kích hoạt.',true);});
async function stopMotion(send=true){const had=!!armKey;invalidateMotion();if(send||had){try{const r=await api('/api/control/stop','POST',{});const acknowledged=r.mux_confirmed===true&&r.cancel_confirmed===true;const message=r.message||r.note||'Đã gửi yêu cầu dừng.';showNotice(acknowledged?message:message+' Đang chờ xác nhận.',acknowledged);}catch(err){showNotice('Chưa xác nhận dừng robot. '+err.message);}}}
$('globalStop').onclick=task(()=>stopMotion());$('driveStop').onclick=task(()=>stopMotion());
$('armButton').onclick=task(async()=>{
  if(armPending||armKey||!statusIsFresh()||latest.ros_ok===false||busyOperation())return;
  const generation=motionGeneration;armPending=true;$('armButton').disabled=true;
  try{
    const r=await api('/api/control/arm','POST',{watchdog_tested:$('watchdogAck').checked,clear_area:$('areaAck').checked});
    if(generation!==motionGeneration||!statusIsFresh()||latest.ros_ok===false)return;
    armKey=r.key;lastArmAt=Date.now();sequence=0;held=[0,0];
  }finally{armPending=false;renderStatus();}
});
function requestedTwist(){
  const radius=latest.limits?.radius_m||.044, track=latest.limits?.track_m||.195;
  const straight=Number($('straightRpm').value), turn=Number($('turnRpm').value);
  return [held[0]*2*Math.PI*radius*straight/60, held[1]*4*Math.PI*radius*turn/(60*track)];
}
let rpmOptionsSignature='';
function syncRpmOptions(){
  const limits=latest.limits||{},max=Number(limits.max_wheel_rpm||40);
  const choices=(Array.isArray(limits.choices_rpm)?limits.choices_rpm:[20,30,40]).filter(v=>Number.isFinite(v)&&v>0&&v<=max);
  const signature=JSON.stringify([choices,max]);if(signature===rpmOptionsSignature||armKey)return;
  const first=!rpmOptionsSignature;rpmOptionsSignature=signature;
  for(const [id,preferred] of [['straightRpm',limits.straight_rpm||40],['turnRpm',limits.turn_rpm||20]]){
    const select=$(id),old=Number(select.value);select.replaceChildren();
    for(const value of choices){const option=document.createElement('option');option.value=String(value);option.textContent=`${value} RPM`;select.append(option);}
    const pick=first?preferred:old;select.value=String(choices.includes(pick)?pick:choices[0]||0);
  }
  text('manualLimitLabel',`Giới hạn: ${max} RPM/bánh.`);
}
function updateSpeedLabel(){
  const radius=latest.limits?.radius_m||.044, track=latest.limits?.track_m||.195;
  const straight=Number($('straightRpm').value), turn=Number($('turnRpm').value);
  const requestedV=2*Math.PI*radius*straight/60,requestedW=4*Math.PI*radius*turn/(60*track);
  text('speedLabel',`Tiến/lùi ${requestedV.toFixed(3)} m/s · Quay ${requestedW.toFixed(3)} rad/s`);
  const mux=latest.navigation?.mux||{},warnings=[];
  if(Number.isFinite(mux.max_linear)&&mux.max_linear>0&&requestedV>mux.max_linear+1e-6)warnings.push(`Giới hạn tiến: ${mux.max_linear.toFixed(3)} m/s (≈ ${(mux.max_linear*60/(2*Math.PI*radius)).toFixed(1)} RPM/bánh).`);
  if(Number.isFinite(mux.max_angular)&&mux.max_angular>0&&requestedW>mux.max_angular+1e-6)warnings.push(`Giới hạn quay: ${mux.max_angular.toFixed(3)} rad/s (≈ ${(mux.max_angular*track*60/(4*Math.PI*radius)).toFixed(1)} RPM/bánh).`);
  $('muxSpeedNotice').hidden=!warnings.length;text('muxSpeedNotice',warnings.join(' '));
}
$('straightRpm').onchange=$('turnRpm').onchange=()=>{held=[0,0];updateSpeedLabel();};
setInterval(async()=>{
  if(!armKey||commandPending||document.hidden||currentTab!=='mapping'||!statusIsFresh()||latest.ros_ok===false)return;
  commandPending=true;const key=armKey;
  const [v,w]=requestedTwist();
  try{await api('/api/control/command','POST',{key,seq:++sequence,v,w,sent_at_ms:Date.now()});}
  catch(err){armKey=null;held=[0,0];showNotice(err.message);}
  finally{commandPending=false;}
},100);
for(const b of document.querySelectorAll('.drive')){
  b.onpointerdown=e=>{if(!armKey)return;e.preventDefault();b.setPointerCapture(e.pointerId);held=[Number(b.dataset.v),Number(b.dataset.w)];};
  b.onpointerup=b.onpointercancel=b.onlostpointercapture=()=>{held=[0,0];};
}
window.addEventListener('keydown',e=>{if(e.code==='Space'&&!/INPUT|TEXTAREA|SELECT/.test(e.target.tagName)){e.preventDefault();stopMotion();return;}if(!armKey||currentTab!=='mapping'||/INPUT|TEXTAREA|SELECT/.test(e.target.tagName))return;const keys={KeyW:[1,0],KeyS:[-1,0],KeyA:[0,1],KeyD:[0,-1]};if(keys[e.code]){e.preventDefault();held=keys[e.code];}});
window.addEventListener('keyup',e=>{if(['KeyW','KeyS','KeyA','KeyD'].includes(e.code))held=[0,0];});
window.addEventListener('blur',()=>{if(armKey)stopMotion();});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&armKey)stopMotion();});
window.addEventListener('pagehide',()=>{if(armKey){fetch('/api/control/stop',{method:'POST',headers:{'Content-Type':'application/json','X-CSRF-Token':csrf},body:'{}',keepalive:true});armKey=null;}});
for(const nav of document.querySelectorAll('[data-tab]'))nav.onclick=task(async()=>{if(armKey)await stopMotion();currentTab=nav.dataset.tab;document.querySelectorAll('.tab-panel').forEach(p=>p.hidden=p.id!==`tab-${currentTab}`);document.querySelectorAll('.nav-item').forEach(n=>n.classList.toggle('active',n===nav));text('pageTitle',titles[currentTab]);if(currentTab==='mapping'){updateLive();liveView.fit();}if(currentTab==='library'){await refreshMaps();savedView.fit();}if(currentTab==='localization'){await refreshMaps();if(selectedLocalizationMapId)await loadLocalizationMap(selectedLocalizationMapId,false);localizationView.fit();renderLocalizationStatus();}document.title=`${titles[currentTab]} · Robot Delivery`;if(currentTab==='logs')refreshAudit();if(currentTab==='delivery'&&typeof enterDelivery==='function')await enterDelivery();});

class MapCanvas{
  constructor(id,{live=false,scanToggle=null,robotToggle=null,pointsProvider=null,selectEnabled=null,pathProvider=null,goalProvider=null,liveGuard=null}={}){this.canvas=$(id);this.ctx=this.canvas.getContext('2d');this.live=live;this.scanToggle=scanToggle;this.robotToggle=robotToggle;this.pointsProvider=pointsProvider;this.selectEnabled=selectEnabled||(()=>false);this.pathProvider=pathProvider;this.goalProvider=goalProvider;this.liveGuard=liveGuard;this.image=null;this.imageRequest=0;this.meta=null;this.zoom=1;this.offset=[0,0];this.drag=null;this.pointerStart=null;this.onSelect=null;this.draft=null;this.poseMarker=null;
    new ResizeObserver(()=>{const r=this.canvas.getBoundingClientRect();if(r.width&&r.height){this.canvas.width=Math.round(r.width);this.canvas.height=Math.round(r.height);this.draw();}}).observe(this.canvas);
    this.canvas.addEventListener('wheel',e=>{e.preventDefault();if(!this.meta)return;const p=this.mouse(e);const old=this.zoom;this.zoom=Math.max(.03,Math.min(30,this.zoom*(e.deltaY<0?1.15:1/1.15)));this.offset=[p[0]-(p[0]-this.offset[0])*this.zoom/old,p[1]-(p[1]-this.offset[1])*this.zoom/old];this.draw();},{passive:false});
    this.canvas.onpointerdown=e=>{if(!this.meta)return;this.canvas.setPointerCapture(e.pointerId);this.pointerStart=this.mouse(e);this.drag=[...this.offset];};
    this.canvas.onpointermove=e=>{if(!this.pointerStart)return;const p=this.mouse(e);if(this.onSelect&&this.selectEnabled()){const a=this.toWorld(this.pointerStart),b=this.toWorld(p);this.draft={x:a[0],y:a[1],yaw:Math.atan2(b[1]-a[1],b[0]-a[0])};}else this.offset=[this.drag[0]+p[0]-this.pointerStart[0],this.drag[1]+p[1]-this.pointerStart[1]];this.draw();};
    this.canvas.onpointerup=e=>{if(!this.pointerStart)return;if(this.onSelect&&this.selectEnabled()){const p=this.mouse(e),a=this.toWorld(this.pointerStart),b=this.toWorld(p);const yaw=Math.hypot(p[0]-this.pointerStart[0],p[1]-this.pointerStart[1])<4?0:Math.atan2(b[1]-a[1],b[0]-a[0]);this.onSelect(a[0],a[1],yaw);}this.pointerStart=this.drag=null;this.draft=null;this.draw();};
    this.canvas.onpointercancel=()=>{this.pointerStart=this.drag=null;this.draft=null;this.draw();};
  }
  mouse(e){const r=this.canvas.getBoundingClientRect();return [e.clientX-r.left,e.clientY-r.top];}
  async set(meta,url,fit=false){const request=++this.imageRequest;const image=new Image();await new Promise((resolve,reject)=>{image.onload=resolve;image.onerror=()=>reject(new Error('Không tải được ảnh map.'));image.src=url;});if(request!==this.imageRequest)return;this.meta=meta;this.image=image;if(fit)this.fit();else this.draw();}
  clear(){this.imageRequest++;this.image=null;this.meta=null;this.draft=null;this.poseMarker=null;this.ctx.clearRect(0,0,this.canvas.width,this.canvas.height);}
  fit(){if(!this.meta)return;const w=this.canvas.width,h=this.canvas.height;this.zoom=Math.max(.03,Math.min((w-50)/this.meta.width,(h-50)/this.meta.height));this.offset=[(w-this.meta.width*this.zoom)/2,(h-this.meta.height*this.zoom)/2];this.draw();}
  toWorld(p){const m=this.meta;const u=(p[0]-this.offset[0])/this.zoom*m.resolution;const v=(m.height-(p[1]-this.offset[1])/this.zoom)*m.resolution;const [ox,oy,a]=m.origin;return [ox+Math.cos(a)*u-Math.sin(a)*v,oy+Math.sin(a)*u+Math.cos(a)*v];}
  toCanvas(x,y){const m=this.meta,[ox,oy,a]=m.origin,dx=x-ox,dy=y-oy;return [this.offset[0]+(Math.cos(a)*dx+Math.sin(a)*dy)/m.resolution*this.zoom,this.offset[1]+(m.height-(-Math.sin(a)*dx+Math.cos(a)*dy)/m.resolution)*this.zoom];}
  arrow(x,y,yaw,color,size=11){const c=this.ctx,p=this.toCanvas(x,y);c.save();c.translate(...p);c.rotate(-(yaw-this.meta.origin[2]));c.fillStyle=color;c.strokeStyle='#fff';c.lineWidth=1.5;c.beginPath();c.moveTo(size,0);c.lineTo(-size*.65,-size*.65);c.lineTo(-size*.35,0);c.lineTo(-size*.65,size*.65);c.closePath();c.fill();c.stroke();c.restore();}
  drawPoints(rows){const c=this.ctx;for(const p of rows||[]){this.arrow(p.x,p.y,p.yaw,p.kind==='pickup'?'#1464ba':p.kind==='delivery'?'#ab5070':'#088b7f',9);const q=this.toCanvas(p.x,p.y);c.font='12px sans-serif';c.fillStyle='#16333d';c.fillText(p.name,q[0]+12,q[1]-9);}}
  draw(){const c=this.ctx;c.clearRect(0,0,this.canvas.width,this.canvas.height);if(!this.image||!this.meta)return;c.imageSmoothingEnabled=false;c.drawImage(this.image,this.offset[0],this.offset[1],this.meta.width*this.zoom,this.meta.height*this.zoom);
    if(this.pathProvider){const path=this.pathProvider();if(Array.isArray(path)&&path.length>1){c.save();c.strokeStyle='#4878d8';c.lineWidth=2.5;c.lineCap='round';c.lineJoin='round';c.beginPath();path.forEach((p,i)=>{const q=this.toCanvas(p[0],p[1]);if(i===0)c.moveTo(...q);else c.lineTo(...q);});c.stroke();c.restore();}}
    if(this.pointsProvider)this.drawPoints(this.pointsProvider());
    if(this.goalProvider){const goal=this.goalProvider();if(goal&&[goal.x,goal.y,goal.yaw].every(Number.isFinite))this.arrow(goal.x,goal.y,goal.yaw,'#4878d8',12);}
    if(this.live&&(!this.liveGuard||this.liveGuard())){const toggle=this.scanToggle?$(this.scanToggle):null;if(!toggle||toggle.checked){c.fillStyle='#078f91';for(const p of latest.scan_points||[]){const q=this.toCanvas(...p);c.fillRect(q[0]-1,q[1]-1,2,2);}}const robotToggle=this.robotToggle?$(this.robotToggle):null;const robot=latest.tf_map;if((!robotToggle||robotToggle.checked)&&robot&&robot.age<1&&robot.age>=-.75)this.arrow(robot.x,robot.y,robot.yaw,'#e26a28',14);}
    if(this.draft)this.arrow(this.draft.x,this.draft.y,this.draft.yaw,'#d6a11d',12);if(this.poseMarker)this.arrow(this.poseMarker.x,this.poseMarker.y,this.poseMarker.yaw,'#d6a11d',12);
  }
}
const liveView=new MapCanvas('liveCanvas',{live:true,scanToggle:'showScan'}),savedView=new MapCanvas('savedCanvas',{pointsProvider:()=>points,selectEnabled:()=>pointMode}),localizationView=new MapCanvas('localizationCanvas',{live:true,scanToggle:'showScanLoc',robotToggle:'showRobotLoc',pointsProvider:()=>($('showPointsLoc').checked?localizationPoints:[]),selectEnabled:()=>initialPoseMode});
$('fitLive').onclick=()=>liveView.fit();$('fitSaved').onclick=()=>savedView.fit();$('fitLocalization').onclick=()=>localizationView.fit();$('showScan').onchange=()=>liveView.draw();$('showScanLoc').onchange=$('showPointsLoc').onchange=$('showRobotLoc').onchange=()=>localizationView.draw();
$('liveMapMode').onchange=()=>{liveMapMode=$('liveMapMode').value;liveRevision=-1;updateLive();};
async function updateLive(){if(liveFetch||!loggedIn)return;liveFetch=true;try{const r=await api('/api/map/live');$('liveEmpty').hidden=r.available;if(!r.available){if(liveRevision!==-1||liveView.image){liveView.clear();liveRevision=-1;}return;}if(r.revision!==liveRevision){const first=liveRevision<0;const imagePath=liveMapMode==='clean'?'/api/map/live-clean.png':'/api/map/live.png';await liveView.set(r.meta,`${imagePath}?r=${r.revision}`,first);liveRevision=r.revision;}}catch(e){showNotice(e.message);}finally{liveFetch=false;}}
$('saveMap').onclick=task(async()=>{if(armKey)await stopMotion();showNotice('Đang lưu map và kết thúc phiên SLAM hiện tại…');const r=await api('/api/maps/save','POST',{name:$('mapName').value,posegraph:$('saveGraph').checked});liveView.clear();liveRevision=-1;$('liveEmpty').hidden=false;showNotice(r.warning||`Đã lưu bản đồ “${r.map.name}”.`,!r.warning);await refreshMaps();});
async function refreshMaps(){mapRows=await api('/api/maps');const list=$('mapList');list.replaceChildren();if(!mapRows.length){const p=document.createElement('p');p.className='muted small';p.textContent='Chưa có bản đồ trong ứng dụng. Nhập map cũ hoặc lưu phiên SLAM.';list.append(p);}for(const m of mapRows){const b=button('',()=>selectMap(m),'map-item'+(selectedMap?.id===m.id?' active':''));const label=document.createElement('span');label.textContent=m.name;const sub=document.createElement('small');sub.textContent=new Date(m.created*1000).toLocaleString('vi-VN')+' · '+m.id.slice(0,8);b.append(label,sub);list.append(b);}const select=$('localizationMap');if(select){const keep=selectedLocalizationMapId||select.value;select.replaceChildren();const first=document.createElement('option');first.value='';first.textContent=mapRows.length?'Chọn map dùng cho giao hàng…':'Chưa có map trong thư viện';select.append(first);for(const m of mapRows){const o=document.createElement('option');o.value=m.id;o.textContent=`${m.name} · ${m.meta.width}×${m.meta.height} · ${m.meta.resolution.toFixed(3)} m`;select.append(o);}if(mapRows.some(m=>m.id===keep)){selectedLocalizationMapId=keep;select.value=keep;}else{selectedLocalizationMapId='';select.value='';}}if(typeof refreshDeliveryMapOptions==='function')refreshDeliveryMapOptions();}
async function selectMap(m){selectedMap=m;editingPoint=null;points=await api(`/api/maps/${m.id}/points`);text('savedMapTitle',m.name);$('savedEmpty').hidden=true;$('exportMap').disabled=false;await savedView.set(m.meta,`/api/maps/${m.id}/preview.png`,true);renderPoints();await refreshMaps();}
$('refreshMaps').onclick=task(refreshMaps);
$('findMaps').onclick=task(async()=>{const rows=await api('/api/maps/import-candidates');const div=$('importList');div.replaceChildren();if(!rows.length)div.textContent='Chưa tìm thấy map hợp lệ trong ~/robot_maps.';for(const row of rows){const b=button(`Nhập ${row.name}`,async()=>{const name=prompt('Tên bản đồ trong ứng dụng',row.name);if(!name)return;await api('/api/maps/import','POST',{key:row.key,name});await refreshMaps();showNotice('Đã nhập bản đồ.',true);},'ghost wide');div.append(b);}});
$('exportMap').onclick=()=>{if(selectedMap)location.href=`/api/maps/${selectedMap.id}/export`;};
$('pointMode').onclick=()=>{pointMode=!pointMode;text('pointMode',`Đặt điểm: ${pointMode?'bật':'tắt'}`);};
savedView.onSelect=(x,y,yaw)=>{$('pointX').value=x.toFixed(3);$('pointY').value=y.toFixed(3);$('pointYaw').value=(yaw*180/Math.PI).toFixed(0);};
$('pointForm').onsubmit=task(async e=>{e.preventDefault();if(!selectedMap)throw new Error('Chọn bản đồ đã lưu trước.');const data={name:$('pointName').value,kind:$('pointKind').value,x:Number($('pointX').value),y:Number($('pointY').value),yaw:Number($('pointYaw').value)*Math.PI/180,note:$('pointNote').value};if(editingPoint)data.id=editingPoint;const r=await api(`/api/maps/${selectedMap.id}/points`,'POST',data);points=await api(`/api/maps/${selectedMap.id}/points`);editingPoint=null;renderPoints();savedView.draw();showNotice(r.note,true);if(typeof refreshDeliveryPoints==='function')await refreshDeliveryPoints();});
$('clearPoint').onclick=()=>{editingPoint=null;$('pointForm').reset();};
function renderPoints(){const div=$('pointList');div.replaceChildren();for(const p of points){const row=document.createElement('div');row.className='point-row';const label=document.createElement('span');label.textContent=`${p.name} · (${p.x.toFixed(2)}, ${p.y.toFixed(2)})`;const controls=document.createElement('span');controls.append(button('Sửa',()=>{editingPoint=p.id;$('pointName').value=p.name;$('pointKind').value=p.kind;$('pointX').value=p.x;$('pointY').value=p.y;$('pointYaw').value=(p.yaw*180/Math.PI).toFixed(1);$('pointNote').value=p.note;}),button('Xóa',async()=>{if(!confirm(`Xóa điểm ${p.name}?`))return;await api(`/api/maps/${selectedMap.id}/points/${p.id}`,'DELETE');points=await api(`/api/maps/${selectedMap.id}/points`);renderPoints();savedView.draw();if(typeof refreshDeliveryPoints==='function')await refreshDeliveryPoints();}));row.append(label,controls);div.append(row);}}
function mapMetaMatches(a,b){if(!a||!b||a.width!==b.width||a.height!==b.height||Math.abs(a.resolution-b.resolution)>=1e-6||((a.frame_id&&b.frame_id)&&a.frame_id!==b.frame_id))return false;const ao=a.origin||[],bo=b.origin||[];return ao.length===3&&bo.length===3&&ao.every((v,i)=>Math.abs(v-bo[i])<1e-5);}
async function setLocalizationMapImage(fit=false){
  const m=mapRows.find(x=>x.id===selectedLocalizationMapId);if(!m)return;
  localizationMapMode=$('localizationMapMode').value;let url,key,label;
  if(localizationMapMode==='clean'){url=`/api/maps/${m.id}/view/clean.png?v=4`;key=`clean:${m.id}:4`;label='HIỂN THỊ SẠCH';}
  else{
    let rosMap=null;try{rosMap=await api('/api/map/ros');}catch(_e){}
    const context=latest.processes?.localization?.context||{};const contextMatches=!context.map_id||context.map_id===m.id;const rosMapServerActive=latest.nodes?.includes('/map_server')||latest.nodes?.includes('/amcl');
    if(rosMap?.available&&rosMapServerActive&&contextMatches&&mapMetaMatches(rosMap.meta,m.meta)){url=`/api/map/ros.png?r=${rosMap.revision}`;key=`ros:${rosMap.revision}`;label='BẢN ĐỒ ROS';}
    else{url=`/api/maps/${m.id}/view/technical.png?v=4`;key=`technical:${m.id}:4`;label='OCCUPANCY';}
  }
  text('mapRenderSource',label);
  if(key!==localizationImageKey||fit){await localizationView.set(m.meta,url,fit);localizationImageKey=key;}else localizationView.draw();
}
async function refreshLocalizationMapSource(){if(localizationMapMode!=='technical'||!selectedLocalizationMapId||localizationMapRefreshBusy)return;const now=Date.now();if(now-lastLocalizationMapCheck<1800)return;lastLocalizationMapCheck=now;localizationMapRefreshBusy=true;try{await setLocalizationMapImage(false);}catch(_e){}finally{localizationMapRefreshBusy=false;}}
async function loadLocalizationMap(id,fit=true){selectedLocalizationMapId=id||'';initialPoseDraft=null;localizationView.poseMarker=null;localizationImageKey='';if(!id){localizationPoints=[];localizationView.clear();$('localizationEmpty').hidden=false;text('localizationMapTitle','Bản đồ định vị');text('mapRenderSource','HIỂN THỊ SẠCH');renderLocalizationStatus();return;}const m=mapRows.find(x=>x.id===id);if(!m)throw new Error('Map đã chọn không còn trong thư viện.');localizationPoints=await api(`/api/maps/${m.id}/points`);$('localizationEmpty').hidden=true;text('localizationMapTitle',`Giám sát · ${m.name}`);await setLocalizationMapImage(fit);text('initialPoseHint','Chưa chọn Initial Pose cho phiên này.');}
$('localizationMap').onchange=task(async()=>{await loadLocalizationMap($('localizationMap').value,true);});
$('localizationMapMode').onchange=task(async()=>{localizationMapMode=$('localizationMapMode').value;localizationImageKey='';await setLocalizationMapImage(false);});
$('gotoImportMap').onclick=()=>{document.querySelector('[data-tab="library"]').click();setTimeout(()=>$('findMaps').click(),80);};
$('startLocalization').onclick=task(async()=>{if(!selectedLocalizationMapId)throw new Error('Chọn map dùng cho giao hàng trước.');await stopMotion(false);const r=await api('/api/localization/start','POST',{map_id:selectedLocalizationMapId});acceptOperation(r,'Đang bật định vị.');localizationImageKey='';setTimeout(()=>{setLocalizationMapImage(false).catch(()=>{});},1200);renderLocalizationStatus();});
$('stopLocalization').onclick=task(async()=>{await stopProcess('localization');renderLocalizationStatus();});
$('startCostmap').onclick=task(async()=>{await startProcess('costmap');renderLocalizationStatus();});
$('stopCostmap').onclick=task(async()=>{await stopProcess('costmap');renderLocalizationStatus();});
$('openRviz').onclick=task(()=>startProcess('rviz'));
$('stopRviz').onclick=task(async()=>{const r=await api('/api/rviz/stop','POST',{});acceptOperation(r,'Đang đóng RViz.');});
$('initialPoseMode').onchange=()=>{initialPoseMode=$('initialPoseMode').checked;$('localizationCanvas').parentElement.classList.toggle('pose-select',initialPoseMode);text('initialPoseHint',initialPoseMode?'Nhấn và kéo để chọn vị trí, hướng.':'Chưa chọn vị trí.');};
localizationView.onSelect=(x,y,yaw)=>{initialPoseDraft={x,y,yaw};localizationView.poseMarker=initialPoseDraft;$('initialX').value=x.toFixed(3);$('initialY').value=y.toFixed(3);$('initialYaw').value=(yaw*180/Math.PI).toFixed(1);text('initialPoseHint',`Đã chọn x=${x.toFixed(2)} m, y=${y.toFixed(2)} m, yaw=${(yaw*180/Math.PI).toFixed(0)}°. Chưa gửi.`);localizationView.draw();};
for(const id of ['initialX','initialY','initialYaw'])$(id).oninput=()=>{const raw=[$('initialX').value,$('initialY').value,$('initialYaw').value];if(raw.some(v=>v.trim()==='')){initialPoseDraft=null;localizationView.poseMarker=null;localizationView.draw();return;}const x=Number(raw[0]),y=Number(raw[1]),yaw=Number(raw[2])*Math.PI/180;if([x,y,yaw].every(Number.isFinite)){initialPoseDraft={x,y,yaw};localizationView.poseMarker=initialPoseDraft;localizationView.draw();}};
$('sendInitialPose').onclick=task(async()=>{if(!selectedLocalizationMapId)throw new Error('Chọn map trước.');const raw=[$('initialX').value,$('initialY').value,$('initialYaw').value];if(raw.some(v=>v.trim()===''))throw new Error('Chọn hoặc nhập đủ X, Y và Yaw.');const x=Number(raw[0]),y=Number(raw[1]),yaw=Number(raw[2])*Math.PI/180;if(![x,y,yaw].every(Number.isFinite))throw new Error('Chọn hoặc nhập Initial Pose hợp lệ.');const r=await api('/api/localization/initial-pose','POST',{map_id:selectedLocalizationMapId,x,y,yaw});initialPoseDraft={x,y,yaw};localizationView.poseMarker=initialPoseDraft;text('initialPoseHint','Đã gửi vị trí. Chờ định vị cập nhật.');showNotice(r.note,true);});
function renderLocalizationStatus(){
  const loc=latest.processes?.localization||{},cost=latest.processes?.costmap||{},rviz=latest.processes?.rviz||{};
  const available=statusIsFresh()&&latest.ros_ok!==false,offline=latest.mode==='OFFLINE',busy=busyOperation();
  const amclNode=available&&latest.nodes?.includes('/amcl'),tf=latest.tf_map;
  const tfGood=available&&tf&&Number.isFinite(tf.age)&&tf.age<1&&tf.age>=-.75;
  text('amclWebState',!available?'Chưa có dữ liệu':amclNode?(loc.external?'Phiên đã nhận diện':'Đang hoạt động'):loc.running?'Đang khởi động':'Chưa chạy');
  text('mapTfWebState',tfGood?`Đang cập nhật · ${formatAge(tf.age)}`:amclNode?'Chờ vị trí ban đầu':'Chưa có');
  text('robotMapPose',tfGood&&[tf.x,tf.y,tf.yaw].every(Number.isFinite)?`x ${tf.x.toFixed(2)} · y ${tf.y.toFixed(2)} · ${(tf.yaw*180/Math.PI).toFixed(0)}°`:'—');
  const scan=sampleDisplay(latest.scan,latest.scan_publishers,available);
  text('localizationScanState',`${scan.value}${scan.fresh?(latest.scan_tf_ready?' · TF sẵn sàng':' · Chờ TF'):''}`);
  const globalActive=available&&(latest.global_costmap_publishers||0)>0,localActive=available&&(latest.local_costmap_publishers||0)>0;
  text('globalCostmapWebState',globalActive?'Đang phát':cost.running?'Chờ dữ liệu':'Chưa chạy');
  text('localCostmapWebState',localActive?'Đang phát':cost.running?'Chờ dữ liệu':'Chưa chạy');
  const stopping=name=>latest.system_operation?.state==='running'&&latest.system_operation?.kind==='component_stop'&&latest.system_operation?.component===name;
  $('startLocalization').disabled=!available||busy||!selectedLocalizationMapId||loc.running||loc.external||latest.nodes?.includes('/slam_toolbox')||offline;
  $('stopLocalization').disabled=offline||!loc.running||(loc.external&&!loc.recoverable)||stopping('localization');
  $('sendInitialPose').disabled=!available||busy||!loc.running||loc.external||!selectedLocalizationMapId;
  $('startCostmap').disabled=!available||busy||!amclNode||!tfGood||cost.running||offline;
  $('stopCostmap').disabled=offline||!cost.running||(cost.external&&!cost.recoverable)||!!cost.context?.provided_by||stopping('costmap');
  $('openRviz').disabled=!available||busy||rviz.running||offline;
  $('stopRviz').disabled=offline||!rviz.running||(rviz.external&&!rviz.recoverable)||stopping('rviz');
  const ctx=loc.context||{};
  if(available&&ctx.map_id&&selectedLocalizationMapId&&ctx.map_id!==selectedLocalizationMapId)text('amclWebState',`Bản đồ đang dùng: ${ctx.map_name||ctx.map_id.slice(0,8)}`);
  refreshLocalizationMapSource();
}

async function loadSettings(){const s=await api('/api/settings');$('lidarIp').value=s.lidar_ip;$('lidarPort').value=s.lidar_port;$('agentPort').value=s.agent_port;}
$('settingsForm').onsubmit=task(async e=>{e.preventDefault();const r=await api('/api/settings','POST',{lidar_ip:$('lidarIp').value,lidar_port:Number($('lidarPort').value),agent_port:Number($('agentPort').value)});showNotice(r.note,true);});
$('refreshPorts').onclick=task(async()=>{const ports=await api('/api/serial/ports');const select=$('serialPort');select.replaceChildren();for(const p of ports){const o=document.createElement('option');o.value=p.device;o.textContent=`${p.device} · ${p.description} · ${p.serial_number||'không có serial'}`;select.append(o);}if(!ports.length){const o=document.createElement('option');o.value='';o.textContent='Ubuntu chưa thấy cổng USB';select.append(o);}});
$('readSerial').onclick=task(async()=>{$('readSerial').disabled=true;try{const r=await api('/api/serial/read','POST',{device:$('serialPort').value,baud:Number($('serialBaud').value),motor_power_off:$('usbSafety').checked});text('serialOutput',r.text);showNotice(r.note);}finally{$('readSerial').disabled=false;}});
$('refreshLog').onclick=task(async()=>{const r=await api(`/api/process/${$('logSelect').value}/log`);text('processLog',r.text);});
async function refreshAudit(){const rows=await api('/api/audit');const div=$('auditList');div.replaceChildren();for(const r of rows){const row=document.createElement('div');row.className='audit-row';row.textContent=`${r.event} ${r.detail}`;const time=document.createElement('span');time.textContent=new Date(r.stamp*1000).toLocaleString('vi-VN');row.append(time);div.append(row);}}
$('refreshAudit').onclick=task(refreshAudit);
api('/api/session').then(enter).catch(()=>{});

$('connectSystem').onclick=task(async()=>{
  if(systemBusy||busyOperation())return;systemBusy=true;$('connectSystem').disabled=true;
  try{await stopMotion(false);acceptOperation(await api('/api/system/connect','POST',{}),'Đang kết nối hệ thống…');}
  finally{systemBusy=false;renderStatus();}
});
$('disconnectSystem').onclick=task(async()=>{
  if(latest.system_operation?.state==='running'&&latest.system_operation?.kind==='disconnect')return;
  invalidateMotion();
  acceptOperation(await api('/api/system/disconnect','POST',{}),'Đang dừng hệ thống…');
  renderStatus();
});
setInterval(()=>{if(loggedIn&&!statusIsFresh()){text('streamState','Chưa có cập nhật');renderStatus();}},1500);
$('inspectProject').onclick=task(async()=>{text('projectOutput',JSON.stringify(await api('/api/project'),null,2));});
$('wifiBoard').onchange=()=>{$('wifiPort').value=$('wifiBoard').value==='esp32_1'?'8888':'7777';text('wifiProbe','Hãy kiểm tra project board vừa chọn.');};
$('probeWifi').onclick=task(async()=>{text('wifiProbe',JSON.stringify(await api(`/api/usb/probe/${$('wifiBoard').value}`),null,2));});
$('wifiForm').onsubmit=task(async e=>{e.preventDefault();if(!confirm('Build đúng project hiện tại với bản sdkconfig riêng. Chưa flash. Tiếp tục?'))return;
 const r=await api(`/api/usb/build/${$('wifiBoard').value}`,'POST',{ssid:$('wifiSsid').value,password:$('wifiPassword').value,host:$('wifiAgent').value,port:Number($('wifiPort').value),motor_power_off:$('wifiSafety').checked});
 $('wifiPassword').value='';showNotice(r.message,true);});
$('flashWifi').onclick=task(async()=>{if(!confirm('Board sẽ RESET. Đã ngắt nguồn motor và xác nhận đúng cổng/board? Không rút USB hoặc đóng web trong lúc nạp.'))return;
 const r=await api('/api/usb/flash','POST',{device:$('serialPort').value,confirmation:$('flashConfirm').value,motor_power_off:$('wifiSafety').checked,board_verified:$('boardVerified').checked});showNotice(r.message);});
$('wifiLogButton').onclick=task(async()=>{text('wifiLog',(await api('/api/usb/log')).text);});

/* WEB_STATUS_SELF_HEAL_V1 */
// WebSocket la nguon chinh; HTTP chi dong bo trang thai khi can.
(() => {
  let trackedSocket = null;
  let lastMessageAt = 0;
  let checking = false;

  function observeStream() {
    if (!loggedIn) {
      trackedSocket = null;
      lastMessageAt = 0;
      return;
    }

    if (trackedSocket !== socket) {
      trackedSocket = socket;
      lastMessageAt = Date.now();

      if (trackedSocket) {
        const observed = trackedSocket;
        observed.addEventListener('message', () => {
          if (socket === observed) lastMessageAt = Date.now();
        });
      }
    }

    if (socket &&
        socket.readyState === WebSocket.OPEN &&
        Date.now() - lastMessageAt > 6000) {
      // onclose hien co se ket noi lai.
      socket.close(4000, 'Status stream stale');
    }
  }

  async function syncStatus() {
    if (!loggedIn || document.hidden || checking) return;
    checking = true;
    const streamTime = lastMessageAt;

    try {
      const r = await fetch('/api/status', {
        credentials: 'same-origin',
        cache: 'no-store',
        signal: AbortSignal.timeout(3500)
      });

      if (!r.ok || !loggedIn) return;

      const status = await r.json();
      if (!status || !status.navigation || !status.processes) return;

      const previous = latest || {};
      const oldOp = previous.navigation_operation || {};
      const newOp = status.navigation_operation || {};

      const streamStale =
        !socket ||
        socket.readyState !== WebSocket.OPEN ||
        Date.now() - lastMessageAt > 3500;

      const changed =
        oldOp.id !== newOp.id ||
        oldOp.state !== newOp.state ||
        previous.navigation?.ready !== status.navigation?.ready ||
        previous.processes?.navigation?.status !==
          status.processes?.navigation?.status ||
        previous.navigation?.mux?.mode !== status.navigation?.mux?.mode;

      // Khong ghi de trang thai WebSocket moi hon.
      if (!streamStale && lastMessageAt > streamTime) return;

      if (changed || streamStale) {
        latest = status;
        renderStatus();

        // HTTP khong thay the ket noi an toan cho lai tay.
        if (streamStale) {
          armKey = null;
          held = [0, 0];
          const armButton = document.getElementById('armButton');
          if (armButton) armButton.disabled = true;
          document.querySelectorAll('.drive').forEach(
            button => button.disabled = true
          );
        }
      }
    } catch (_err) {
      // Khong lam gian doan WebSocket chinh.
    } finally {
      checking = false;
    }
  }

  setInterval(() => {
    observeStream();
    void syncStatus();
  }, 2000);
})();
