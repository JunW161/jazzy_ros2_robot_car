'use strict';
const $ = id => document.getElementById(id);
let csrf = '', socket = null, currentTab = 'overview', loggedIn = false;
let latest = {}, mapRows = [], selectedMap = null, points = [], editingPoint = null;
let selectedLocalizationMapId = '', localizationPoints = [], initialPoseMode = false, initialPoseDraft = null;
let lastArmAt = 0, systemBusy = false;
let armKey = null, sequence = 0, held = [0,0], commandPending = false, liveRevision = -1;
let liveFetch = false, pointMode = false;
let liveMapMode = 'clean', localizationMapMode = 'clean', localizationImageKey = '';
let localizationMapRefreshBusy = false, lastLocalizationMapCheck = 0;
const titles = {overview:'Tổng quan hệ thống',mapping:'Tạo bản đồ & điều khiển',library:'Bản đồ & điểm giao nhận',localization:'Định vị & giám sát realtime',devices:'Thiết bị và cấu hình USB',logs:'Nhật ký hệ thống'};
function showNotice(text, good=false) { $('notice').textContent=text; $('notice').classList.toggle('good',good); $('notice').hidden=false; }
async function api(path, method='GET', body) {
  const response=await fetch(path,{method,credentials:'same-origin',headers:{'Content-Type':'application/json','X-CSRF-Token':csrf},body:body===undefined?undefined:JSON.stringify(body)});
  const data=await response.json().catch(()=>({detail:'Không đọc được phản hồi.'}));
  if(!response.ok) throw new Error(typeof data.detail==='string'?data.detail:JSON.stringify(data.detail));
  return data;
}
function task(fn){return async event=>{try{await fn(event);}catch(err){showNotice(err.message);}};}
function text(id, value){$(id).textContent=value;}
function button(label,fn,style='ghost'){const b=document.createElement('button'); b.textContent=label;b.className=style;b.addEventListener('click',task(fn));return b;}
async function enter(session){csrf=session.csrf;loggedIn=true;$('loginView').hidden=true;$('workspace').hidden=false;$('password').value='';connect();await loadSettings();await refreshMaps();}
$('loginForm').addEventListener('submit',async e=>{e.preventDefault();$('loginError').textContent='';try{await enter(await api('/api/login','POST',{username:$('username').value,password:$('password').value}));}catch(err){text('loginError',err.message);}});
$('logoutButton').onclick=task(async()=>{await stopMotion();await api('/api/logout','POST',{});loggedIn=false;socket?.close();$('workspace').hidden=true;$('loginView').hidden=false;csrf='';});
function connect(){
  if(!loggedIn)return;
  socket=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/ws/status`);
  socket.onopen=()=>text('streamState','Backend đã kết nối');
  socket.onmessage=e=>{latest=JSON.parse(e.data);renderStatus();};
  socket.onerror=()=>text('streamState','Lỗi kết nối');
  socket.onclose=()=>{armKey=null;held=[0,0];text('streamState','Mất kết nối · điều khiển khóa');if(loggedIn)setTimeout(connect,2000);};
}
function renderStatus(){
  const s=latest;
  text('modeLabel',s.mode==='OFFLINE'?'OFFLINE · chỉ kiểm tra giao diện':'Kết nối ROS 2 trên Ubuntu');
  text('odomHz',`${(s.odom?.hz||0).toFixed(1)} Hz`);text('scanHz',`${(s.scan?.hz||0).toFixed(1)} Hz`);
  const age=v=>v==null?'chưa có stamp':`${(v*1000).toFixed(0)} ms`;
  text('odomAge',`Publisher: ${s.odom_publishers} · tuổi ${age(s.odom?.stamp_age)}`);
  text('scanAge',`Publisher: ${s.scan_publishers} · tuổi ${age(s.scan?.stamp_age)}`);
  text('tfState',s.tf_odom?'Có dữ liệu':'Chưa có');text('tfAge',`Tuổi TF: ${age(s.tf_odom?.age)}`);
  text('slamState',s.slam_state||'offline');text('mapState',s.map_revision?'Đã nhận /map':'Chưa nhận bản đồ thực');
  text('systemBadge',s.motion?.armed?'Đang giữ quyền điều khiển':'Chỉ quan sát');
  text('motionState',s.motion?.reason||'Chưa bật điều khiển.');
  if(!s.motion?.armed&&armKey&&Date.now()-lastArmAt>500){armKey=null;held=[0,0];}
  $('armButton').disabled=!!armKey||!s.motion_ready;
  document.querySelectorAll('.drive').forEach(b=>b.disabled=!armKey);
  const checks=$('readiness');checks.replaceChildren();
  const issues=s.mapping_issues||[];
  for(const line of issues.length?issues:['Dữ liệu và TF đạt kiểm tra đầu vào. Có thể thử SLAM có giám sát.']){const item=document.createElement('div');item.className='checkline'+(issues.length?'':' ok');item.textContent=(issues.length?'!  ':'✓  ')+line;checks.append(item);}
  if(s.other_cmd_publishers?.length){const el=document.createElement('div');el.className='checkline';el.textContent='Nguồn /cmd_vel khác: '+s.other_cmd_publishers.join(', ')+' — web khóa điều khiển.';checks.append(el);}
  $('startSlam').disabled=!!issues.length||s.nodes?.includes('/slam_toolbox');
  $('activateSlam').disabled=s.slam_state!=='inactive'||!s.processes?.slam?.running;
  $('stopSlam').disabled=!s.processes?.slam?.running;
  $('saveMap').disabled=!s.map_revision||s.slam_state!=='active'||!!s.motion?.armed;
  const m=s.motor||{};
  const n=v=>v==null?'—':Number(v).toFixed(1);
  text('motorStatus',`Trạng thái: ${m.state_name||'—'} · Interlock: ${m.interlock??'—'} · RPM trái/phải: ${n(m.rpm_l)} / ${n(m.rpm_r)} · Yaw: ${n(m.yaw_deg)}° · Heading: ${m.heading_enabled==null?'—':m.heading_enabled?'ON':'OFF'}`);
  const mi=$('motionIssues');mi.replaceChildren();
  for(const reason of s.motion_issues||[]){const item=document.createElement('div');item.className='checkline';item.textContent=reason;mi.append(item);}
  const u=s.usb_job||{};
  text('wifiState',`${u.state||'idle'} · ${u.message||'Không có công việc USB.'}`);
  $('buildWifi').disabled=!!u.busy||s.mode==='OFFLINE';
  $('flashWifi').disabled=u.state!=='built'||!!u.busy||s.mode==='OFFLINE';
  $('connectSystem').disabled=s.mode==='OFFLINE'||!!u.busy||systemBusy;
  $('disconnectSystem').disabled=s.mode==='OFFLINE'||!!u.busy||systemBusy;
  $('straightRpm').disabled=!!armKey; $('turnRpm').disabled=!!armKey;
  const rpm=n;
  text('driveTelemetry',`Đặt firmware L/R: ${rpm(m.target_l)} / ${rpm(m.target_r)} RPM · Đo L/R: ${rpm(m.rpm_l)} / ${rpm(m.rpm_r)} RPM · PWM: ${rpm(m.pwm_l)} / ${rpm(m.pwm_r)} · ${m.state_name||'—'} / khóa ${m.interlock??'—'}`);
  updateSpeedLabel();
  renderProcesses();
  if(currentTab==='mapping') { updateLive();liveView.draw(); }
  if(currentTab==='localization') { renderLocalizationStatus(); localizationView?.draw(); }
}
const processNames={agent:'micro-ROS Agent',odom_bridge:'Odom Bridge (joint only)',ekf_odom:'EKF V14 chính thức',robot_model:'Mô hình robot',lidar:'LiDAR Wi-Fi',slam:'SLAM Toolbox'};
const processNodes={agent:'/delivery_robot_motor',lidar:'/ldlidar_publisher_ld14',odom_bridge:'/delivery_odom_tf_joint_bridge',ekf_odom:'/ekf_filter_node_v14',robot_model:'/robot_state_publisher',slam:'/slam_toolbox'};
let processSignature='';
function renderProcesses(){
  const sig=JSON.stringify([latest.processes,latest.nodes]);if(sig===processSignature)return;processSignature=sig;
  const root=$('processCards');root.replaceChildren();
  for(const [key,label] of Object.entries(processNames)){
    const own=latest.processes?.[key];const external=own?.external||(latest.nodes?.includes(processNodes[key])&&!own?.running);
    const card=document.createElement('div');card.className='process-card';
    const h=document.createElement('h4');h.textContent=label;card.append(h);
    const p=document.createElement('p');p.textContent=own?.message||(external?'Đang chạy bên ngoài web; dừng Terminal cũ một lần rồi dùng nút web.':own?.running?`Web quản lý · PID ${own.pid}`:own?`Đã kết thúc · mã ${own.returncode}`:'Chưa do web khởi động');card.append(p);
    const row=document.createElement('div');row.className='button-row';
    const start=button('Khởi động',()=>startProcess(key),'primary');start.disabled=latest.mode==='OFFLINE'||!!own?.running||external;
    const stop=button('Dừng',()=>stopProcess(key));stop.disabled=!own?.running||(external&&!own?.recoverable);
    const log=button('Xem log',async()=>{document.querySelector('[data-tab="logs"]').click();$('logSelect').value=key;text('processLog',(await api(`/api/process/${key}/log`)).text);});
    row.append(start,stop,log);card.append(row);root.append(card);
  }
}
async function startProcess(name){await stopMotion(false);showNotice(`Đang khởi động ${processNames[name]}…`);const r=await api(`/api/process/${name}/start`,'POST',{});showNotice(r.message,true);}
async function stopProcess(name){if(!confirm(`Dừng ${processNames[name]} do web quản lý? Robot sẽ được khóa điều khiển.`))return;await stopMotion();const r=await api(`/api/process/${name}/stop`,'POST',{});showNotice(r.message,true);}
$('startSlam').onclick=task(()=>startProcess('slam'));$('stopSlam').onclick=task(()=>stopProcess('slam'));
$('activateSlam').onclick=task(async()=>{await api('/api/slam/activate','POST',{});showNotice('Đã gửi activate. Theo dõi trạng thái SLAM.',true);});
async function stopMotion(send=true){const had=!!armKey;armKey=null;held=[0,0];if(send||had){try{await api('/api/control/stop','POST',{});}catch(err){showNotice('Không gửi được lệnh dừng; phải dựa vào watchdog ESP32/nút dừng vật lý. '+err.message);}}}
$('globalStop').onclick=task(()=>stopMotion());$('driveStop').onclick=task(()=>stopMotion());
$('armButton').onclick=task(async()=>{const r=await api('/api/control/arm','POST',{watchdog_tested:$('watchdogAck').checked,clear_area:$('areaAck').checked});armKey=r.key;lastArmAt=Date.now();sequence=0;held=[0,0];$('armButton').disabled=true;document.querySelectorAll('.drive').forEach(b=>b.disabled=false);});
function requestedTwist(){
  const radius=latest.limits?.radius_m||.044, track=latest.limits?.track_m||.195;
  const straight=Number($('straightRpm').value), turn=Number($('turnRpm').value);
  return [held[0]*2*Math.PI*radius*straight/60, held[1]*4*Math.PI*radius*turn/(60*track)];
}
function updateSpeedLabel(){
  const radius=latest.limits?.radius_m||.044, track=latest.limits?.track_m||.195;
  const straight=Number($('straightRpm').value), turn=Number($('turnRpm').value);
  text('speedLabel',`Tiến/lùi ${straight} RPM ≈ ${(2*Math.PI*radius*straight/60).toFixed(3)} m/s. Quay ${turn} RPM/bánh ≈ ${(4*Math.PI*radius*turn/(60*track)).toFixed(3)} rad/s. Lệnh đặt, không phải RPM đo.`);
}
$('straightRpm').onchange=$('turnRpm').onchange=()=>{held=[0,0];updateSpeedLabel();};
setInterval(async()=>{
  if(!armKey||commandPending||document.hidden||currentTab!=='mapping')return;
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
for(const nav of document.querySelectorAll('[data-tab]'))nav.onclick=task(async()=>{if(armKey)await stopMotion();currentTab=nav.dataset.tab;document.querySelectorAll('.tab-panel').forEach(p=>p.hidden=p.id!==`tab-${currentTab}`);document.querySelectorAll('.nav-item').forEach(n=>n.classList.toggle('active',n===nav));text('pageTitle',titles[currentTab]);if(currentTab==='mapping'){updateLive();liveView.fit();}if(currentTab==='library'){await refreshMaps();savedView.fit();}if(currentTab==='localization'){await refreshMaps();if(selectedLocalizationMapId)await loadLocalizationMap(selectedLocalizationMapId,false);localizationView.fit();renderLocalizationStatus();}if(currentTab==='logs')refreshAudit();});

class MapCanvas{
  constructor(id,{live=false,scanToggle=null,robotToggle=null,pointsProvider=null,selectEnabled=null}={}){this.canvas=$(id);this.ctx=this.canvas.getContext('2d');this.live=live;this.scanToggle=scanToggle;this.robotToggle=robotToggle;this.pointsProvider=pointsProvider;this.selectEnabled=selectEnabled||(()=>false);this.image=null;this.meta=null;this.zoom=1;this.offset=[0,0];this.drag=null;this.pointerStart=null;this.onSelect=null;this.draft=null;this.poseMarker=null;
    new ResizeObserver(()=>{const r=this.canvas.getBoundingClientRect();if(r.width&&r.height){this.canvas.width=Math.round(r.width);this.canvas.height=Math.round(r.height);this.draw();}}).observe(this.canvas);
    this.canvas.addEventListener('wheel',e=>{e.preventDefault();if(!this.meta)return;const p=this.mouse(e);const old=this.zoom;this.zoom=Math.max(.03,Math.min(30,this.zoom*(e.deltaY<0?1.15:1/1.15)));this.offset=[p[0]-(p[0]-this.offset[0])*this.zoom/old,p[1]-(p[1]-this.offset[1])*this.zoom/old];this.draw();},{passive:false});
    this.canvas.onpointerdown=e=>{if(!this.meta)return;this.canvas.setPointerCapture(e.pointerId);this.pointerStart=this.mouse(e);this.drag=[...this.offset];};
    this.canvas.onpointermove=e=>{if(!this.pointerStart)return;const p=this.mouse(e);if(this.onSelect&&this.selectEnabled()){const a=this.toWorld(this.pointerStart),b=this.toWorld(p);this.draft={x:a[0],y:a[1],yaw:Math.atan2(b[1]-a[1],b[0]-a[0])};}else this.offset=[this.drag[0]+p[0]-this.pointerStart[0],this.drag[1]+p[1]-this.pointerStart[1]];this.draw();};
    this.canvas.onpointerup=e=>{if(!this.pointerStart)return;if(this.onSelect&&this.selectEnabled()){const p=this.mouse(e),a=this.toWorld(this.pointerStart),b=this.toWorld(p);const yaw=Math.hypot(p[0]-this.pointerStart[0],p[1]-this.pointerStart[1])<4?0:Math.atan2(b[1]-a[1],b[0]-a[0]);this.onSelect(a[0],a[1],yaw);}this.pointerStart=this.drag=null;this.draft=null;this.draw();};
    this.canvas.onpointercancel=()=>{this.pointerStart=this.drag=null;this.draft=null;this.draw();};
  }
  mouse(e){const r=this.canvas.getBoundingClientRect();return [e.clientX-r.left,e.clientY-r.top];}
  async set(meta,url,fit=false){const image=new Image();await new Promise((resolve,reject)=>{image.onload=resolve;image.onerror=()=>reject(new Error('Không tải được ảnh map.'));image.src=url;});this.meta=meta;this.image=image;if(fit)this.fit();else this.draw();}
  clear(){this.image=null;this.meta=null;this.draft=null;this.poseMarker=null;this.ctx.clearRect(0,0,this.canvas.width,this.canvas.height);}
  fit(){if(!this.meta)return;const w=this.canvas.width,h=this.canvas.height;this.zoom=Math.max(.03,Math.min((w-50)/this.meta.width,(h-50)/this.meta.height));this.offset=[(w-this.meta.width*this.zoom)/2,(h-this.meta.height*this.zoom)/2];this.draw();}
  toWorld(p){const m=this.meta;const u=(p[0]-this.offset[0])/this.zoom*m.resolution;const v=(m.height-(p[1]-this.offset[1])/this.zoom)*m.resolution;const [ox,oy,a]=m.origin;return [ox+Math.cos(a)*u-Math.sin(a)*v,oy+Math.sin(a)*u+Math.cos(a)*v];}
  toCanvas(x,y){const m=this.meta,[ox,oy,a]=m.origin,dx=x-ox,dy=y-oy;return [this.offset[0]+(Math.cos(a)*dx+Math.sin(a)*dy)/m.resolution*this.zoom,this.offset[1]+(m.height-(-Math.sin(a)*dx+Math.cos(a)*dy)/m.resolution)*this.zoom];}
  arrow(x,y,yaw,color,size=11){const c=this.ctx,p=this.toCanvas(x,y);c.save();c.translate(...p);c.rotate(-(yaw-this.meta.origin[2]));c.fillStyle=color;c.strokeStyle='#fff';c.lineWidth=1.5;c.beginPath();c.moveTo(size,0);c.lineTo(-size*.65,-size*.65);c.lineTo(-size*.35,0);c.lineTo(-size*.65,size*.65);c.closePath();c.fill();c.stroke();c.restore();}
  drawPoints(rows){const c=this.ctx;for(const p of rows||[]){this.arrow(p.x,p.y,p.yaw,p.kind==='pickup'?'#1464ba':p.kind==='delivery'?'#ab5070':'#088b7f',9);const q=this.toCanvas(p.x,p.y);c.font='12px sans-serif';c.fillStyle='#16333d';c.fillText(p.name,q[0]+12,q[1]-9);}}
  draw(){const c=this.ctx;c.clearRect(0,0,this.canvas.width,this.canvas.height);if(!this.image||!this.meta)return;c.imageSmoothingEnabled=false;c.drawImage(this.image,this.offset[0],this.offset[1],this.meta.width*this.zoom,this.meta.height*this.zoom);
    if(this.pointsProvider)this.drawPoints(this.pointsProvider());
    if(this.live){const toggle=this.scanToggle?$(this.scanToggle):null;if(!toggle||toggle.checked){c.fillStyle='#078f91';for(const p of latest.scan_points||[]){const q=this.toCanvas(...p);c.fillRect(q[0]-1,q[1]-1,2,2);}}const robotToggle=this.robotToggle?$(this.robotToggle):null;const robot=latest.tf_map;if((!robotToggle||robotToggle.checked)&&robot&&robot.age<1&&robot.age>-.15)this.arrow(robot.x,robot.y,robot.yaw,'#e26a28',14);}
    if(this.draft)this.arrow(this.draft.x,this.draft.y,this.draft.yaw,'#d6a11d',12);if(this.poseMarker)this.arrow(this.poseMarker.x,this.poseMarker.y,this.poseMarker.yaw,'#d6a11d',12);
  }
}
const liveView=new MapCanvas('liveCanvas',{live:true,scanToggle:'showScan'}),savedView=new MapCanvas('savedCanvas',{pointsProvider:()=>points,selectEnabled:()=>pointMode}),localizationView=new MapCanvas('localizationCanvas',{live:true,scanToggle:'showScanLoc',robotToggle:'showRobotLoc',pointsProvider:()=>($('showPointsLoc').checked?localizationPoints:[]),selectEnabled:()=>initialPoseMode});
$('fitLive').onclick=()=>liveView.fit();$('fitSaved').onclick=()=>savedView.fit();$('fitLocalization').onclick=()=>localizationView.fit();$('showScan').onchange=()=>liveView.draw();$('showScanLoc').onchange=$('showPointsLoc').onchange=$('showRobotLoc').onchange=()=>localizationView.draw();
$('liveMapMode').onchange=()=>{liveMapMode=$('liveMapMode').value;liveRevision=-1;updateLive();};
async function updateLive(){if(liveFetch||!loggedIn)return;liveFetch=true;try{const r=await api('/api/map/live');$('liveEmpty').hidden=r.available;if(!r.available){if(liveRevision!==-1||liveView.image){liveView.clear();liveRevision=-1;}return;}if(r.revision!==liveRevision){const first=liveRevision<0;const imagePath=liveMapMode==='clean'?'/api/map/live-clean.png':'/api/map/live.png';await liveView.set(r.meta,`${imagePath}?r=${r.revision}`,first);liveRevision=r.revision;}}catch(e){showNotice(e.message);}finally{liveFetch=false;}}
$('saveMap').onclick=task(async()=>{if(armKey)await stopMotion();showNotice('Đang lưu map và kết thúc phiên SLAM hiện tại…');const r=await api('/api/maps/save','POST',{name:$('mapName').value,posegraph:$('saveGraph').checked});liveView.clear();liveRevision=-1;$('liveEmpty').hidden=false;showNotice(r.warning||`Đã lưu bản đồ “${r.map.name}” thành phiên bản ${r.map.id.slice(0,8)} và xóa map LIVE của phiên cũ.`,!r.warning);await refreshMaps();});
async function refreshMaps(){mapRows=await api('/api/maps');const list=$('mapList');list.replaceChildren();if(!mapRows.length){const p=document.createElement('p');p.className='muted small';p.textContent='Chưa có bản đồ trong ứng dụng. Nhập map cũ hoặc lưu phiên SLAM.';list.append(p);}for(const m of mapRows){const b=button('',()=>selectMap(m),'map-item'+(selectedMap?.id===m.id?' active':''));const label=document.createElement('span');label.textContent=m.name;const sub=document.createElement('small');sub.textContent=new Date(m.created*1000).toLocaleString('vi-VN')+' · '+m.id.slice(0,8);b.append(label,sub);list.append(b);}const select=$('localizationMap');if(select){const keep=selectedLocalizationMapId||select.value;select.replaceChildren();const first=document.createElement('option');first.value='';first.textContent=mapRows.length?'Chọn map dùng cho giao hàng…':'Chưa có map trong thư viện';select.append(first);for(const m of mapRows){const o=document.createElement('option');o.value=m.id;o.textContent=`${m.name} · ${m.meta.width}×${m.meta.height} · ${m.meta.resolution.toFixed(3)} m`;select.append(o);}if(mapRows.some(m=>m.id===keep)){selectedLocalizationMapId=keep;select.value=keep;}else{selectedLocalizationMapId='';select.value='';}}}
async function selectMap(m){selectedMap=m;editingPoint=null;points=await api(`/api/maps/${m.id}/points`);text('savedMapTitle',m.name);$('savedEmpty').hidden=true;$('exportMap').disabled=false;await savedView.set(m.meta,`/api/maps/${m.id}/preview.png`,true);renderPoints();await refreshMaps();}
$('refreshMaps').onclick=task(refreshMaps);
$('findMaps').onclick=task(async()=>{const rows=await api('/api/maps/import-candidates');const div=$('importList');div.replaceChildren();if(!rows.length)div.textContent='Chưa tìm thấy map hợp lệ trong ~/robot_maps.';for(const row of rows){const b=button(`Nhập ${row.name}`,async()=>{const name=prompt('Tên bản đồ trong ứng dụng',row.name);if(!name)return;await api('/api/maps/import','POST',{key:row.key,name});await refreshMaps();showNotice('Đã nhập bản sao map. Không sửa file gốc.',true);},'ghost wide');div.append(b);}});
$('exportMap').onclick=()=>{if(selectedMap)location.href=`/api/maps/${selectedMap.id}/export`;};
$('pointMode').onclick=()=>{pointMode=!pointMode;text('pointMode',`Đặt điểm: ${pointMode?'bật':'tắt'}`);};
savedView.onSelect=(x,y,yaw)=>{$('pointX').value=x.toFixed(3);$('pointY').value=y.toFixed(3);$('pointYaw').value=(yaw*180/Math.PI).toFixed(0);};
$('pointForm').onsubmit=task(async e=>{e.preventDefault();if(!selectedMap)throw new Error('Chọn bản đồ đã lưu trước.');const data={name:$('pointName').value,kind:$('pointKind').value,x:Number($('pointX').value),y:Number($('pointY').value),yaw:Number($('pointYaw').value)*Math.PI/180,note:$('pointNote').value};if(editingPoint)data.id=editingPoint;const r=await api(`/api/maps/${selectedMap.id}/points`,'POST',data);points=await api(`/api/maps/${selectedMap.id}/points`);editingPoint=null;renderPoints();savedView.draw();showNotice(r.note,true);});
$('clearPoint').onclick=()=>{editingPoint=null;$('pointForm').reset();};
function renderPoints(){const div=$('pointList');div.replaceChildren();for(const p of points){const row=document.createElement('div');row.className='point-row';const label=document.createElement('span');label.textContent=`${p.name} · (${p.x.toFixed(2)}, ${p.y.toFixed(2)})`;const controls=document.createElement('span');controls.append(button('Sửa',()=>{editingPoint=p.id;$('pointName').value=p.name;$('pointKind').value=p.kind;$('pointX').value=p.x;$('pointY').value=p.y;$('pointYaw').value=(p.yaw*180/Math.PI).toFixed(1);$('pointNote').value=p.note;}),button('Xóa',async()=>{if(!confirm(`Xóa điểm ${p.name}?`))return;await api(`/api/maps/${selectedMap.id}/points/${p.id}`,'DELETE');points=await api(`/api/maps/${selectedMap.id}/points`);renderPoints();savedView.draw();}));row.append(label,controls);div.append(row);}}
function mapMetaMatches(a,b){if(!a||!b||a.width!==b.width||a.height!==b.height||Math.abs(a.resolution-b.resolution)>=1e-6||((a.frame_id&&b.frame_id)&&a.frame_id!==b.frame_id))return false;const ao=a.origin||[],bo=b.origin||[];return ao.length===3&&bo.length===3&&ao.every((v,i)=>Math.abs(v-bo[i])<1e-5);}
async function setLocalizationMapImage(fit=false){
  const m=mapRows.find(x=>x.id===selectedLocalizationMapId);if(!m)return;
  localizationMapMode=$('localizationMapMode').value;let url,key,label;
  if(localizationMapMode==='clean'){url=`/api/maps/${m.id}/view/clean.png?v=4`;key=`clean:${m.id}:4`;label='CLEAN VIEW · chỉ hiển thị';}
  else{
    let rosMap=null;try{rosMap=await api('/api/map/ros');}catch(_e){}
    const context=latest.processes?.localization?.context||{};const contextMatches=!context.map_id||context.map_id===m.id;const rosMapServerActive=latest.nodes?.includes('/map_server')||latest.nodes?.includes('/amcl');
    if(rosMap?.available&&rosMapServerActive&&contextMatches&&mapMetaMatches(rosMap.meta,m.meta)){url=`/api/map/ros.png?r=${rosMap.revision}`;key=`ros:${rosMap.revision}`;label='ROS /map · cùng nguồn RViz';}
    else{url=`/api/maps/${m.id}/view/technical.png?v=4`;key=`technical:${m.id}:4`;label='YAML TECHNICAL · tương đương occupancy';}
  }
  text('mapRenderSource',label);
  if(key!==localizationImageKey||fit){await localizationView.set(m.meta,url,fit);localizationImageKey=key;}else localizationView.draw();
}
async function refreshLocalizationMapSource(){if(localizationMapMode!=='technical'||!selectedLocalizationMapId||localizationMapRefreshBusy)return;const now=Date.now();if(now-lastLocalizationMapCheck<1800)return;lastLocalizationMapCheck=now;localizationMapRefreshBusy=true;try{await setLocalizationMapImage(false);}catch(_e){}finally{localizationMapRefreshBusy=false;}}
async function loadLocalizationMap(id,fit=true){selectedLocalizationMapId=id||'';initialPoseDraft=null;localizationView.poseMarker=null;localizationImageKey='';if(!id){localizationPoints=[];localizationView.clear();$('localizationEmpty').hidden=false;text('localizationMapTitle','Định vị trên map đã lưu');text('mapRenderSource','CLEAN VIEW');renderLocalizationStatus();return;}const m=mapRows.find(x=>x.id===id);if(!m)throw new Error('Map đã chọn không còn trong thư viện.');localizationPoints=await api(`/api/maps/${m.id}/points`);$('localizationEmpty').hidden=true;text('localizationMapTitle',`Giám sát · ${m.name}`);await setLocalizationMapImage(fit);text('initialPoseHint','Chưa chọn Initial Pose cho phiên này.');}
$('localizationMap').onchange=task(async()=>{await loadLocalizationMap($('localizationMap').value,true);});
$('localizationMapMode').onchange=task(async()=>{localizationMapMode=$('localizationMapMode').value;localizationImageKey='';await setLocalizationMapImage(false);});
$('gotoImportMap').onclick=()=>{document.querySelector('[data-tab="library"]').click();setTimeout(()=>$('findMaps').click(),80);};
$('startLocalization').onclick=task(async()=>{if(!selectedLocalizationMapId)throw new Error('Chọn map dùng cho giao hàng trước.');await stopMotion(false);const r=await api('/api/localization/start','POST',{map_id:selectedLocalizationMapId});showNotice(r.message,true);localizationImageKey='';setTimeout(()=>{setLocalizationMapImage(false).catch(()=>{});},1200);renderLocalizationStatus();});
$('stopLocalization').onclick=task(async()=>{if(!confirm('Dừng Map Server + AMCL? Web chỉ dừng tiến trình Localization do Web quản lý hoặc đúng standalone launch của dự án đã nhận diện. Robot sẽ mất localization trên map.'))return;await stopMotion();const r=await api('/api/localization/stop','POST',{});showNotice(r.message,true);renderLocalizationStatus();});
$('startCostmap').onclick=task(async()=>{await stopMotion(false);const r=await api('/api/process/costmap/start','POST',{});showNotice(r.message,true);renderLocalizationStatus();});
$('stopCostmap').onclick=task(async()=>{if(!confirm('Dừng Global + Local Costmap Nav2?'))return;await stopMotion(false);const r=await api('/api/process/costmap/stop','POST',{});showNotice(r.message,true);renderLocalizationStatus();});
$('openRviz').onclick=task(async()=>{const r=await api('/api/rviz/start','POST',{});showNotice(r.message,true);});
$('stopRviz').onclick=task(async()=>{const r=await api('/api/rviz/stop','POST',{});showNotice(r.message,true);});
$('initialPoseMode').onchange=()=>{initialPoseMode=$('initialPoseMode').checked;$('localizationCanvas').parentElement.classList.toggle('pose-select',initialPoseMode);text('initialPoseHint',initialPoseMode?'Nhấn và kéo trên map để chọn vị trí + hướng robot.':'Chế độ chọn Initial Pose đang tắt.');};
localizationView.onSelect=(x,y,yaw)=>{initialPoseDraft={x,y,yaw};localizationView.poseMarker=initialPoseDraft;$('initialX').value=x.toFixed(3);$('initialY').value=y.toFixed(3);$('initialYaw').value=(yaw*180/Math.PI).toFixed(1);text('initialPoseHint',`Đã chọn x=${x.toFixed(2)} m, y=${y.toFixed(2)} m, yaw=${(yaw*180/Math.PI).toFixed(0)}°. Chưa gửi.`);localizationView.draw();};
for(const id of ['initialX','initialY','initialYaw'])$(id).oninput=()=>{const raw=[$('initialX').value,$('initialY').value,$('initialYaw').value];if(raw.some(v=>v.trim()==='')){initialPoseDraft=null;localizationView.poseMarker=null;localizationView.draw();return;}const x=Number(raw[0]),y=Number(raw[1]),yaw=Number(raw[2])*Math.PI/180;if([x,y,yaw].every(Number.isFinite)){initialPoseDraft={x,y,yaw};localizationView.poseMarker=initialPoseDraft;localizationView.draw();}};
$('sendInitialPose').onclick=task(async()=>{if(!selectedLocalizationMapId)throw new Error('Chọn map trước.');const raw=[$('initialX').value,$('initialY').value,$('initialYaw').value];if(raw.some(v=>v.trim()===''))throw new Error('Chọn hoặc nhập đủ X, Y và Yaw.');const x=Number(raw[0]),y=Number(raw[1]),yaw=Number(raw[2])*Math.PI/180;if(![x,y,yaw].every(Number.isFinite))throw new Error('Chọn hoặc nhập Initial Pose hợp lệ.');const r=await api('/api/localization/initial-pose','POST',{map_id:selectedLocalizationMapId,x,y,yaw});initialPoseDraft={x,y,yaw};localizationView.poseMarker=initialPoseDraft;text('initialPoseHint','Đã gửi. Chờ AMCL tạo map→odom; robot sẽ xuất hiện/cập nhật trên map.');showNotice(r.note,true);});
function renderLocalizationStatus(){if(!$('amclWebState'))return;const loc=latest.processes?.localization||{};const running=!!loc.running;const external=!!loc.external;const recoverable=!!loc.recoverable;const amclNode=latest.nodes?.includes('/amcl');text('amclWebState',amclNode?(external?(recoverable?'ACTIVE · có thể dừng':'ACTIVE · ngoài Web'):'ACTIVE'):(running?'Đang khởi động…':'Chưa chạy'));const tf=latest.tf_map;const tfGood=tf&&tf.age<1&&tf.age>-.15;text('mapTfWebState',tfGood?`OK · ${Math.round(tf.age*1000)} ms`:(amclNode?'Chờ Initial Pose':'Chưa có'));text('robotMapPose',tfGood?`x ${tf.x.toFixed(2)} · y ${tf.y.toFixed(2)} · ${(tf.yaw*180/Math.PI).toFixed(0)}°`:'—');text('localizationScanState',`${(latest.scan?.hz||0).toFixed(1)} Hz · ${latest.scan_tf_ready?'TF stamp OK':'chờ TF'}`);const cost=latest.processes?.costmap||{};const costRunning=!!cost.running;const costExternal=!!cost.external;const costRecoverable=!!cost.recoverable;const globalActive=(latest.global_costmap_publishers||0)>0;const localActive=(latest.local_costmap_publishers||0)>0;const costSuffix=costExternal?(costRecoverable?' · có thể dừng':' · ngoài Web'):'';text('globalCostmapWebState',globalActive?`ACTIVE${costSuffix}`:(costRunning?`Đang khởi động${costSuffix}`:'Chưa chạy'));text('localCostmapWebState',localActive?`ACTIVE${costSuffix}`:(costRunning?`Đang khởi động${costSuffix}`:'Chưa chạy'));$('startLocalization').disabled=!selectedLocalizationMapId||running||external||latest.nodes?.includes('/slam_toolbox')||latest.mode==='OFFLINE';$('stopLocalization').disabled=!running||(external&&!recoverable);$('sendInitialPose').disabled=!running||external||!selectedLocalizationMapId;$('startCostmap').disabled=!amclNode||!tfGood||costRunning||latest.mode==='OFFLINE';$('stopCostmap').disabled=!costRunning||(costExternal&&!costRecoverable);$('openRviz').disabled=!!latest.processes?.rviz?.running||latest.mode==='OFFLINE';$('stopRviz').disabled=!latest.processes?.rviz?.running||latest.processes?.rviz?.external;const ctx=loc.context||{};if(ctx.map_id&&selectedLocalizationMapId&&ctx.map_id!==selectedLocalizationMapId){text('amclWebState',`ACTIVE · map khác: ${ctx.map_name||ctx.map_id.slice(0,8)}`);}refreshLocalizationMapSource();}

async function loadSettings(){const s=await api('/api/settings');$('lidarIp').value=s.lidar_ip;$('lidarPort').value=s.lidar_port;$('agentPort').value=s.agent_port;}
$('settingsForm').onsubmit=task(async e=>{e.preventDefault();const r=await api('/api/settings','POST',{lidar_ip:$('lidarIp').value,lidar_port:Number($('lidarPort').value),agent_port:Number($('agentPort').value)});showNotice(r.note,true);});
$('refreshPorts').onclick=task(async()=>{const ports=await api('/api/serial/ports');const select=$('serialPort');select.replaceChildren();for(const p of ports){const o=document.createElement('option');o.value=p.device;o.textContent=`${p.device} · ${p.description} · ${p.serial_number||'không có serial'}`;select.append(o);}if(!ports.length){const o=document.createElement('option');o.value='';o.textContent='Ubuntu chưa thấy cổng USB';select.append(o);}});
$('readSerial').onclick=task(async()=>{$('readSerial').disabled=true;try{const r=await api('/api/serial/read','POST',{device:$('serialPort').value,baud:Number($('serialBaud').value),motor_power_off:$('usbSafety').checked});text('serialOutput',r.text);showNotice(r.note);}finally{$('readSerial').disabled=false;}});
$('refreshLog').onclick=task(async()=>{const r=await api(`/api/process/${$('logSelect').value}/log`);text('processLog',r.text);});
async function refreshAudit(){const rows=await api('/api/audit');const div=$('auditList');div.replaceChildren();for(const r of rows){const row=document.createElement('div');row.className='audit-row';row.textContent=`${r.event} ${r.detail}`;const time=document.createElement('span');time.textContent=new Date(r.stamp*1000).toLocaleString('vi-VN');row.append(time);div.append(row);}}
$('refreshAudit').onclick=task(refreshAudit);
api('/api/session').then(enter).catch(()=>{});

$('connectSystem').onclick=task(async()=>{
 if(systemBusy)return;systemBusy=true;$('connectSystem').disabled=true;await stopMotion(false);
 showNotice('Đang khởi động đúng thứ tự: Agent → Odom(no TF) → EKF V14 → Mô hình robot → LiDAR. Theo dõi trạng thái và log bên dưới.');
 try{const r=await api('/api/system/connect','POST',{});showNotice(r.results.map(x=>`${x.ok?'✓':'!'} ${x.component}: ${x.message}`).join(' | '),r.results.every(x=>x.ok));}
 finally{systemBusy=false;renderStatus();}
});
$('disconnectSystem').onclick=task(async()=>{
 if(!confirm('Dừng và khóa robot, rồi dừng các thành phần do web quản lý? Các tiến trình ngoài web sẽ không bị tắt.'))return;
 systemBusy=true;await stopMotion();
 try{const r=await api('/api/system/disconnect','POST',{});showNotice(r.results.map(x=>`${x.component}: ${x.message}`).join(' | ')||'Không có tiến trình do web quản lý đang chạy.',r.results.every(x=>x.ok));}
 finally{systemBusy=false;renderStatus();}
});
$('inspectProject').onclick=task(async()=>{text('projectOutput',JSON.stringify(await api('/api/project'),null,2));});
$('wifiBoard').onchange=()=>{$('wifiPort').value=$('wifiBoard').value==='esp32_1'?'8888':'7777';text('wifiProbe','Hãy kiểm tra project board vừa chọn.');};
$('probeWifi').onclick=task(async()=>{text('wifiProbe',JSON.stringify(await api(`/api/usb/probe/${$('wifiBoard').value}`),null,2));});
$('wifiForm').onsubmit=task(async e=>{e.preventDefault();if(!confirm('Build đúng project hiện tại với bản sdkconfig riêng. Chưa flash. Tiếp tục?'))return;
 const r=await api(`/api/usb/build/${$('wifiBoard').value}`,'POST',{ssid:$('wifiSsid').value,password:$('wifiPassword').value,host:$('wifiAgent').value,port:Number($('wifiPort').value),motor_power_off:$('wifiSafety').checked});
 $('wifiPassword').value='';showNotice(r.message,true);});
$('flashWifi').onclick=task(async()=>{if(!confirm('Board sẽ RESET. Đã ngắt nguồn motor và xác nhận đúng cổng/board? Không rút USB hoặc đóng web trong lúc nạp.'))return;
 const r=await api('/api/usb/flash','POST',{device:$('serialPort').value,confirmation:$('flashConfirm').value,motor_power_off:$('wifiSafety').checked,board_verified:$('boardVerified').checked});showNotice(r.message);});
$('wifiLogButton').onclick=task(async()=>{text('wifiLog',(await api('/api/usb/log')).text);});
