'use strict';
/* Delivery console. All motion starts are explicit authenticated API requests. */
const missionLabels = {
  IDLE:'Chưa có nhiệm vụ', GOING_TO_PICKUP:'Đang đi lấy hàng', WAITING_PICKUP:'Chờ xếp hàng',
  GOING_TO_DROPOFF:'Đang đi giao hàng', WAITING_DROPOFF:'Chờ nhận hàng',
  COMPLETED:'Đã hoàn tất', CANCELED:'Đã hủy', CANCELING:'Đang hủy', ERROR:'Cần xử lý lỗi',
  UNKNOWN:'Chưa xác định', INTERRUPTED:'Đã gián đoạn'
};
const goalLabels = {IDLE:'Chưa có mục tiêu',SENDING:'Đang gửi mục tiêu',ACTIVE:'Đang di chuyển',
  ACCEPTED:'Đã nhận mục tiêu',CANCELING:'Đang hủy mục tiêu',CANCELED:'Đã hủy mục tiêu',
  UNKNOWN:'Chưa xác nhận kết quả',SUCCEEDED:'Đã đến điểm',FAILED:'Điều hướng thất bại',REJECTED:'Mục tiêu bị từ chối',ERROR:'Lỗi điều hướng'};
const serverLabels = {planner_server:'Lập đường',controller_server:'Bám đường',behavior_server:'Phục hồi',
  bt_navigator:'Điều phối Nav2',waypoint_follower:'Điểm dừng',velocity_smoother:'Làm mượt',collision_monitor:'Giám sát vật cản'};
const deliveryPending = new Set();
let deliveryMapId='', deliveryPoints=[], deliveryMapLoad=0, deliveryLastStatus=0;
let deliveryMapCheck={map_id:'',matches:false,message:'Chưa xác nhận bản đồ đang định vị.',at:0};
let deliveryMapCheckBusy=false, deliveryHistoryBusy=false, deliveryHistoryAt=0, deliveryImageMode='';
let deliveryLastMapIdentity='', deliveryHistorySignature='';

function deliveryStatusReceived(){deliveryLastStatus=Date.now();}
function deliveryDisconnected(){deliveryLastStatus=0;renderDelivery();}
function deliveryConnected(){return loggedIn && socket?.readyState===WebSocket.OPEN && Date.now()-deliveryLastStatus<3500;}
function navData(){return latest.navigation||{};}
function missionData(){return navData().mission||{};}
function missionIsActive(m=missionData()){
  return !!m.active||['GOING_TO_PICKUP','WAITING_PICKUP','GOING_TO_DROPOFF','WAITING_DROPOFF','CANCELING'].includes(m.state_name);
}
function directGoalIsActive(){const goal=navData().goal||{};return !!goal.active||['SENDING','ACCEPTED','ACTIVE','CANCELING','UNKNOWN'].includes(goal.state);}
function deliveryMapMatches(){
  const known=navData().active_map_id||latest.processes?.localization?.context?.map_id;
  if(known && known!==deliveryMapId)return false;
  return !!deliveryMapId && deliveryMapCheck.map_id===deliveryMapId && deliveryMapCheck.matches && Date.now()-deliveryMapCheck.at<6500;
}
function deliveryLiveVisible(){return deliveryConnected()&&deliveryMapMatches()&&latest.mode!=='OFFLINE'&&latest.ros_ok!==false;}
function missionFresh(m=missionData()){
  const age=m.receive_age;
  return latest.ros_ok!==false && m.available===true && Number.isFinite(age)&&age>=0&&age<3;
}
function deliveryReady(){return deliveryConnected() && latest.mode!=='OFFLINE' && navData().ready===true && latest.ros_ok!==false && deliveryMapMatches();}
function stateBadge(id,label,kind='neutral'){
  const el=$(id);el.textContent=label;el.className='badge '+kind;
}
function deliveryPose(){
  const p=latest.tf_map;
  return deliveryLiveVisible()&&p&&[p.x,p.y,p.yaw,p.age].every(Number.isFinite)&&p.age<1&&p.age>=-.75?p:null;
}
function currentPath(){
  const p=navData().plan;
  if(!$('deliveryShowPath').checked||!deliveryLiveVisible()||p?.frame_id!=='map')return [];
  if(!(missionIsActive()||directGoalIsActive()))return [];
  if((p.receive_age??p.age)!=null && (p.receive_age??p.age)>5)return [];
  return (p.points||[]).slice(0,600).filter(q=>Array.isArray(q)&&q.length>=2&&Number.isFinite(q[0])&&Number.isFinite(q[1]));
}
function currentGoalMarker(){
  if(!deliveryLiveVisible()||!directGoalIsActive())return null;
  const p=navData().goal?.pose;
  if(p&&[p.x,p.y,p.yaw].every(Number.isFinite))return p;
  return null;
}
const deliveryView=new MapCanvas('deliveryCanvas',{
  live:true,scanToggle:'deliveryShowScan',pointsProvider:()=>deliveryPoints,
  pathProvider:currentPath,goalProvider:currentGoalMarker,liveGuard:deliveryLiveVisible
});

function refreshDeliveryMapOptions(){
  const select=$('deliveryMap');
  const contextId=navData().active_map_id||latest.processes?.localization?.context?.map_id||'';
  const keep=deliveryMapId||(mapRows.some(m=>m.id===contextId)?contextId:'');
  select.replaceChildren();
  const first=document.createElement('option');first.value='';first.textContent=mapRows.length?'Chọn bản đồ vận hành…':'Chưa có bản đồ đã lưu';select.append(first);
  for(const m of mapRows){const option=document.createElement('option');option.value=m.id;option.textContent=m.name;select.append(option);}
  if(mapRows.some(m=>m.id===keep)){
    select.value=keep;
    if(deliveryMapId!==keep)loadDeliveryMap(keep).catch(err=>showNotice(err.message));
  }else if(deliveryMapId){loadDeliveryMap('').catch(err=>showNotice(err.message));}
}
function pointOptions(id,kind,placeholder){
  const select=$(id),old=select.value;select.replaceChildren();
  const first=document.createElement('option');first.value='';first.textContent=placeholder;select.append(first);
  for(const p of deliveryPoints.filter(p=>!kind||p.kind==='both'||p.kind===kind)){
    const option=document.createElement('option');option.value=p.id;option.textContent=p.name;select.append(option);
  }
  if([...select.options].some(o=>o.value===old))select.value=old;
}
function updateDeliveryPoints(){
  pointOptions('pickupPoint','pickup','Chọn điểm lấy hàng…');
  pointOptions('dropoffPoint','delivery','Chọn điểm giao hàng…');
  pointOptions('singleGoalPoint',null,'Chọn điểm đến…');
  deliveryView.draw();renderDelivery();
}
async function refreshDeliveryPoints(){
  if(!deliveryMapId)return;
  const selected=deliveryMapId, rows=await api(`/api/maps/${encodeURIComponent(selected)}/points`);
  if(selected!==deliveryMapId)return;
  deliveryPoints=Array.isArray(rows)?rows:[];updateDeliveryPoints();
}
async function loadDeliveryMap(id){
  const sequence=++deliveryMapLoad;deliveryMapId=id||'';deliveryPoints=[];
  deliveryMapCheck={map_id:id||'',matches:false,message:'Đang đối chiếu bản đồ đang định vị…',at:0};
  deliveryImageMode='';deliveryView.clear();updateDeliveryPoints();
  $('deliveryMap').value=deliveryMapId;
  if(!id){
    $('deliveryEmpty').hidden=false;text('deliveryMapTitle','Bản đồ vận hành');
    text('deliveryMapSubtitle','Chọn bản đồ vận hành.');renderDelivery();return;
  }
  const row=mapRows.find(m=>m.id===id);
  if(!row)throw new Error('Bản đồ không còn trong thư viện. Hãy làm mới danh sách.');
  text('deliveryMapTitle',row.name);
  text('deliveryMapSubtitle',`${row.meta.width} × ${row.meta.height} ô · Độ phân giải ${row.meta.resolution.toFixed(3)} m`);
  const rows=await api(`/api/maps/${encodeURIComponent(id)}/points`);
  if(sequence!==deliveryMapLoad)return;
  deliveryPoints=Array.isArray(rows)?rows:[];
  await updateDeliveryMapImage(true);
  if(sequence!==deliveryMapLoad)return;
  $('deliveryEmpty').hidden=true;updateDeliveryPoints();await refreshDeliveryMapMatch(true);
}
async function updateDeliveryMapImage(fit=false){
  const row=mapRows.find(m=>m.id===deliveryMapId);if(!row)return;
  const mode=$('deliveryMapMode').value, key=`${row.id}:${mode}`;
  if(deliveryImageMode===key&&!fit){deliveryView.draw();return;}
  deliveryImageMode=key;
  try{await deliveryView.set(row.meta,`/api/maps/${encodeURIComponent(row.id)}/view/${mode}.png`,fit);}
  catch(err){deliveryImageMode='';throw err;}
  text('deliveryMapSource',mode==='clean'?'HIỂN THỊ SẠCH':'OCCUPANCY KỸ THUẬT');
}
async function refreshDeliveryMapMatch(force=false){
  if(!deliveryMapId||deliveryMapCheckBusy||!loggedIn)return;
  if(!force&&Date.now()-deliveryMapCheck.at<2500)return;
  if(latest.mode==='OFFLINE'){
    deliveryMapCheck={map_id:deliveryMapId,matches:false,message:'OFFLINE · chưa có bản đồ định vị từ robot.',at:Date.now()};renderDelivery();return;
  }
  deliveryMapCheckBusy=true;const selected=deliveryMapId;
  try{
    const result=await api(`/api/navigation/map-match/${encodeURIComponent(selected)}`);
    if(selected===deliveryMapId)deliveryMapCheck={map_id:selected,matches:result.matches===true,message:result.message||'',at:Date.now()};
  }catch(err){if(selected===deliveryMapId)deliveryMapCheck={map_id:selected,matches:false,message:err.message,at:Date.now()};}
  finally{deliveryMapCheckBusy=false;renderDelivery();}
}
function renderLifecycle(){
  const data=navData().lifecycle||{};const root=$('navLifecycle');
  root.replaceChildren();
  for(const [key,label] of Object.entries(serverLabels)){
    const raw=data[key]??data['/'+key];
    const state=latest.ros_ok===false||!deliveryConnected()?'unavailable':typeof raw==='string'?raw:(raw?.label||raw?.state||'unavailable');
    const item=document.createElement('div');item.className='lifecycle-item'+(state==='active'?' active':['error','finalized'].includes(state)?' failed':'');
    item.title=key;
    const name=document.createElement('strong');name.textContent=label;
    const status=document.createElement('span');status.textContent=({active:'Sẵn sàng',inactive:'Chưa kích hoạt',unconfigured:'Chưa cấu hình',unavailable:'Chưa có',unknown:'Chưa xác định',finalized:'Đã kết thúc'}[state]||String(state));
    item.append(name,status);root.append(item);
  }
}
function renderDelivery(){
  const n=navData(),m=missionData(),proc=latest.processes?.navigation||{};
  const online=deliveryConnected(),offline=latest.mode==='OFFLINE'||n.offline===true;
  const operation=latest.navigation_operation||{};
  const busy=operation.state==='running'||latest.system_operation?.state==='running'||deliveryPending.has('navigation');
  const active=missionIsActive(m),goalActive=directGoalIsActive(),fresh=missionFresh(m)&&online;
  const externalGoal=n.external_goal_active===true&&!active;
  const ready=deliveryReady(),manual=!!latest.motion?.armed;
  const lifecycleReady=n.ready===true&&online&&!offline&&latest.ros_ok!==false;
  $('offlineBanner').hidden=!offline;
  const navLabel=offline?'OFFLINE':!online?'Mất kết nối':busy?'Đang xử lý':lifecycleReady?'Sẵn sàng':proc.running?'Chưa sẵn sàng':'Chưa khởi động';
  text('navSummary',navLabel);stateBadge('navigationBadge',navLabel.toUpperCase(),lifecycleReady?'success':busy?'warn':'neutral');
  $('navSummaryDot').className='status-dot'+(lifecycleReady?' good':busy||proc.running?' warn':'');
  const mLabel=missionLabels[m.state_name]||'Chưa có nhiệm vụ';
  text('missionSummary',fresh?mLabel:(m.mission_id||m.id?'Mất phản hồi':'Chưa có nhiệm vụ'));
  const mode=n.mux?.mode;const modes={STOP:'Đang dừng',MANUAL:'Điều khiển tay',NAVIGATION:'Điều hướng'};
  text('muxSummary',online&&n.mux?.available?(modes[mode]||mode||'Chưa xác định'):'Chưa có dữ liệu');
  const pose=deliveryPose();
  text('deliveryPose',pose?`${pose.x.toFixed(2)}, ${pose.y.toFixed(2)} m`:'Chờ định vị');
  text('deliveryTelemetry',pose?`x ${pose.x.toFixed(2)} · y ${pose.y.toFixed(2)} · ${(pose.yaw*180/Math.PI).toFixed(0)}°`:'Chưa có vị trí robot');
  const matched=deliveryMapMatches();
  stateBadge('deliveryMapBadge',!deliveryMapId?'CHƯA CHỌN MAP':matched?'ĐÚNG MAP ĐỊNH VỊ':'CHỜ ĐỐI CHIẾU',matched?'success':'neutral');
  const warn=$('deliveryMapWarning');warn.hidden=!deliveryMapId||matched;
  warn.textContent=deliveryMapCheck.message||'Đang đối chiếu bản đồ.';
  if(proc.external)text('navigationDescription','Navigation đang chạy trong phiên khác.');
  else if(lifecycleReady)text('navigationDescription','Sẵn sàng nhận nhiệm vụ.');
  else text('navigationDescription','Kết nối robot và định vị trước khi bật.');
  $('startNavigation').textContent=proc.running&&!fresh?'Kết nối nhiệm vụ':'Bật Navigation';
  $('startNavigation').disabled=!online||offline||latest.ros_ok===false||busy||manual||(lifecycleReady&&fresh)||active||goalActive||externalGoal;
  $('stopNavigation').disabled=offline||deliveryPending.has('navigation')||(operation.state==='running'&&operation.kind==='stop')||!(proc.running||operation.state==='running')||(!!proc.external&&!proc.recoverable);
  $('stopNavigation').title=proc.external&&!proc.recoverable?'Chưa xác định được tiến trình sở hữu.':'Dừng Navigation và quyền tự hành';
  $('navigationOperation').hidden=!operation.state||operation.state==='idle';
  $('navigationOperation').textContent=[operation.message,...(operation.warnings||[])].filter(Boolean).join('\n')||({running:'Đang xử lý…',succeeded:'Đã xử lý yêu cầu.',failed:'Thao tác chưa hoàn thành.'}[operation.state]||'');
  $('navigationOperation').className='operation-status'+(operation.state==='failed'?' failed':'');
  const pickup=$('pickupPoint').value,dropoff=$('dropoffPoint').value;
  const pointsChosen=!!pickup&&!!dropoff&&pickup!==dropoff;
  const commandBusy=deliveryPending.has('mission')||deliveryPending.has('goal')||n.command_pending===true;
  $('startMission').disabled=!ready||!fresh||active||goalActive||externalGoal||manual||busy||commandBusy||!pointsChosen;
  $('pickupPoint').disabled=$('dropoffPoint').disabled=active||goalActive||commandBusy;
  $('deliveryMap').disabled=active||goalActive||commandBusy;
  let hint='Sẵn sàng tạo nhiệm vụ.';
  if(offline)hint='Ngoại tuyến: điều khiển đang khóa.';
  else if(!online)hint='Chờ kết nối trạng thái từ robot.';
  else if(!deliveryMapId)hint='Chọn bản đồ và hai điểm giao nhận.';
  else if(!matched)hint='Bản đồ chưa khớp định vị.';
  else if(!n.ready)hint='Navigation chưa sẵn sàng.';
  else if(!fresh)hint='Quản lý nhiệm vụ chưa kết nối.';
  else if(active)hint='Đang thực hiện một nhiệm vụ.';
  else if(externalGoal)hint='Có mục tiêu từ phiên điều khiển khác.';
  else if(goalActive)hint='Đang đi đến một điểm.';
  else if(manual)hint='Dừng điều khiển tay trước khi bắt đầu giao hàng.';
  else if(!pointsChosen)hint=pickup&&pickup===dropoff?'Điểm lấy và điểm giao phải khác nhau.':'Chọn điểm lấy hàng và điểm giao hàng.';
  text('missionCreateHint',hint);
  const badgeKind=m.state_name==='COMPLETED'?'success':m.state_name==='ERROR'?'danger':['WAITING_PICKUP','WAITING_DROPOFF'].includes(m.state_name)?'warn':'neutral';
  stateBadge('missionStatusBadge',fresh?mLabel.toUpperCase():(m.mission_id||m.id?'MẤT PHẢN HỒI':'SẴN SÀNG CHỜ'),fresh?badgeKind:'neutral');
  text('missionRoute',m.pickup_name&&m.dropoff_name?`${m.pickup_name} → ${m.dropoff_name}`:'Chưa có chuyến giao hàng');
  const ordered=['GOING_TO_PICKUP','WAITING_PICKUP','GOING_TO_DROPOFF','WAITING_DROPOFF'];
  const idx=ordered.indexOf(m.state_name);
  for(const [i,el] of [...$('missionTimeline').children].entries()){
    el.classList.toggle('current',fresh&&i===idx);el.classList.toggle('complete',fresh&&(m.state_name==='COMPLETED'||idx>i));
    el.classList.toggle('error',m.state_name==='ERROR');
  }
  const distance=m.distance_remaining;
  text('missionDistance',fresh&&['GOING_TO_PICKUP','GOING_TO_DROPOFF'].includes(m.state_name)&&Number.isFinite(distance)?`Còn ${distance.toFixed(2)} m`:m.state_name==='COMPLETED'?'Hoàn tất':'—');
  text('missionUpdate',fresh?'Đang nhận phản hồi':m.mission_id||m.id?'Phản hồi đã cũ':'Chưa có phản hồi');
  text('missionMessage',m.message||'Chưa có cập nhật nhiệm vụ.');
  $('confirmPickup').hidden=m.state_name!=='WAITING_PICKUP';
  $('confirmDropoff').hidden=m.state_name!=='WAITING_DROPOFF';
  $('confirmPickup').disabled=!online||!fresh||commandBusy||!ready;
  $('confirmDropoff').disabled=!online||!fresh||commandBusy||latest.ros_ok===false;
  $('cancelMission').disabled=offline||!active||deliveryPending.has('cancel');
  $('resetMission').disabled=!online||!fresh||active||!['COMPLETED','CANCELED','ERROR','INTERRUPTED'].includes(m.state_name)||commandBusy;
  $('sendSingleGoal').disabled=!ready||active||goalActive||externalGoal||manual||busy||commandBusy||!$('singleGoalPoint').value;
  $('singleGoalPoint').disabled=active||goalActive||commandBusy;
  $('cancelSingleGoal').disabled=offline||!goalActive||deliveryPending.has('cancel');
  const goal=n.goal||{};let goalText=goalLabels[goal.state]||'Chưa có mục tiêu.';
  if(goalActive&&Number.isFinite(goal.distance_remaining))goalText+=` · Còn ${goal.distance_remaining.toFixed(2)} m`;
  if(externalGoal)goalText='Có mục tiêu từ phiên điều khiển khác.';
  else if(goal.message&&goal.state!=='IDLE')goalText+=' · '+goal.message;
  text('singleGoalStatus',goalText);
  renderLifecycle();
  text('navigationReadiness',offline?'Chưa kết nối ROS.':!online?'Chưa có cập nhật trạng thái.':latest.ros_ok===false?'Kết nối ROS bị gián đoạn.':n.ready?'Sẵn sàng điều hướng.':'Chưa đủ điều kiện điều hướng.');
  const issues=$('navigationIssues');issues.replaceChildren();
  const messages=[...(n.issues||[])];
  if(deliveryMapId&&!matched&&!offline)messages.push(deliveryMapCheck.message||'Chưa xác nhận bản đồ định vị.');
  if(!online&&!offline)messages.unshift('Mất cập nhật trạng thái. Điều khiển đang khóa.');
  for(const reason of [...new Set(messages)].slice(0,10)){
    const item=document.createElement('div');item.className='navigation-issue';item.textContent=String(reason);issues.append(item);
  }
  if(!messages.length&&lifecycleReady){const el=document.createElement('div');el.className='navigation-issue success';el.textContent='Sẵn sàng nhận mục tiêu.';issues.append(el);}
  if(currentTab==='delivery'){deliveryView.draw();const identity=n.active_map_id||latest.processes?.localization?.context?.map_id||'';
    if(identity!==deliveryLastMapIdentity){deliveryLastMapIdentity=identity;if(!identity||identity!==deliveryMapCheck.map_id){deliveryMapCheck.matches=false;deliveryMapCheck.at=0;}}
  }
  // Preserve the standalone costmap tools, but avoid duplicate Nav2 costmaps.
  if(proc.running||n.ready){$('startCostmap').disabled=true;$('startLocalization').disabled=true;}
}
async function deliveryAction(kind,fn){
  if(deliveryPending.has(kind))return;
  deliveryPending.add(kind);renderDelivery();
  try{return await fn();}
  finally{deliveryPending.delete(kind);renderDelivery();}
}
function jumpDeliveryTab(tab){document.querySelector(`.nav-item[data-tab="${tab}"]`)?.click();}
async function enterDelivery(){
  if(!loggedIn)return;
  refreshDeliveryMapOptions();
  if(deliveryMapId){await refreshDeliveryPoints();await refreshDeliveryMapMatch(true);deliveryView.fit();}
  await refreshMissionHistory();renderDelivery();
}
function formatHistoryTime(value){
  if(value==null||value==='')return '—';
  const date=new Date(typeof value==='number'?(value<1e12?value*1000:value):value);
  return Number.isNaN(date.getTime())?'—':date.toLocaleString('vi-VN',{day:'2-digit',month:'2-digit',year:'numeric',hour:'2-digit',minute:'2-digit'});
}
async function refreshMissionHistory(){
  if(deliveryHistoryBusy||!loggedIn)return;
  deliveryHistoryBusy=true;
  try{
    const data=await api('/api/missions/history');const rows=Array.isArray(data)?data:(data.items||[]);
    const signature=JSON.stringify(rows);deliveryHistoryAt=Date.now();
    if(signature===deliveryHistorySignature)return;deliveryHistorySignature=signature;
    const root=$('missionHistory');root.replaceChildren();
    if(!rows.length){const tr=document.createElement('tr'),td=document.createElement('td');td.colSpan=4;td.className='history-empty';td.textContent='Chưa có nhiệm vụ.';tr.append(td);root.append(tr);return;}
    for(const row of rows.slice(0,50)){
      const tr=document.createElement('tr');
      const stamp=document.createElement('td');stamp.textContent=formatHistoryTime(row.created_at??row.created??row.updated_at);
      const id=document.createElement('td');id.textContent=(row.mission_id||row.id||'—').slice(0,10);const map=document.createElement('small');map.textContent=row.map_name||'';id.append(map);
      const route=document.createElement('td');route.textContent=`${row.pickup_name||'—'} → ${row.dropoff_name||'—'}`;
      const note=document.createElement('small');note.textContent=row.message||'';route.append(note);
      const result=document.createElement('td'),badge=document.createElement('span');const state=row.state_name||row.status||'UNKNOWN';
      badge.className='badge '+(state==='COMPLETED'?'success':['ERROR','FAILED'].includes(state)?'danger':['WAITING_PICKUP','WAITING_DROPOFF'].includes(state)?'warn':'neutral');
      badge.textContent=missionLabels[state]||state;result.append(badge);tr.append(stamp,id,route,result);root.append(tr);
    }
  }catch(err){
    deliveryHistoryAt=Date.now();
    if(!deliveryHistorySignature){const td=$('missionHistory').querySelector('td');if(td)td.textContent='Chưa tải được lịch sử: '+err.message;}
  }finally{deliveryHistoryBusy=false;}
}
$('deliveryMap').onchange=task(()=>loadDeliveryMap($('deliveryMap').value));
$('deliveryMapMode').onchange=task(()=>updateDeliveryMapImage());
$('deliveryShowPath').onchange=$('deliveryShowScan').onchange=()=>deliveryView.draw();
$('fitDelivery').onclick=()=>deliveryView.fit();
$('refreshDeliveryMaps').onclick=task(async()=>{await refreshMaps();await refreshDeliveryPoints();await refreshDeliveryMapMatch(true);});
$('deliverySetup').onclick=task(async()=>{jumpDeliveryTab('localization');if(deliveryMapId){$('localizationMap').value=deliveryMapId;await loadLocalizationMap(deliveryMapId,true);}});
$('deliveryPoints').onclick=task(async()=>{jumpDeliveryTab('library');const map=mapRows.find(m=>m.id===deliveryMapId);if(map)await selectMap(map);});
$('navigationLogs').onclick=task(async()=>{jumpDeliveryTab('logs');$('logSelect').value='navigation';text('processLog',(await api('/api/process/navigation/log')).text);});
$('startNavigation').onclick=task(()=>deliveryAction('navigation',async()=>{
  const r=await api('/api/navigation/start','POST',{});
  showNotice(r.operation?.message||r.message||'Đang bật Navigation.',r.operation?.state==='succeeded');
  if(r.operation)latest.navigation_operation=r.operation;
}));
$('stopNavigation').onclick=task(()=>deliveryAction('navigation',async()=>{
  if(typeof invalidateMotion==='function')invalidateMotion();
  const r=await api('/api/navigation/stop','POST',{});
  showNotice(r.operation?.message||r.message||'Đang dừng Navigation.',r.operation?.state==='succeeded');
  if(r.operation)latest.navigation_operation=r.operation;
}));
$('pickupPoint').onchange=$('dropoffPoint').onchange=$('singleGoalPoint').onchange=()=>renderDelivery();
$('missionForm').onsubmit=task(async event=>{
  event.preventDefault();
  if($('startMission').disabled)return;
  await deliveryAction('mission',async()=>{
    const r=await api('/api/missions/start','POST',{map_id:deliveryMapId,pickup_id:$('pickupPoint').value,dropoff_id:$('dropoffPoint').value});
    showNotice(r.message||'Đã tiếp nhận nhiệm vụ.',true);
    await refreshMissionHistory();
  });
});
for(const [id,action] of [['confirmPickup','confirm_pickup'],['confirmDropoff','confirm_dropoff'],['resetMission','reset']]){
  $(id).onclick=task(()=>deliveryAction('mission',async()=>{
    const r=await api(`/api/missions/${action}`,'POST',{});showNotice(r.message||'Đã nhận yêu cầu.',true);
    await refreshMissionHistory();
  }));
}
$('cancelMission').onclick=task(()=>deliveryAction('cancel',async()=>{
  const r=await api('/api/missions/cancel','POST',{});
  const confirmed=r.success===true&&r.cancel_confirmed===true;
  showNotice(r.message||(confirmed?'Đã xác nhận hủy nhiệm vụ.':'Đã gửi yêu cầu hủy. Đang chờ xác nhận.'),confirmed);
  await refreshMissionHistory();
}));
$('sendSingleGoal').onclick=task(()=>deliveryAction('goal',async()=>{
  const r=await api('/api/navigation/goal','POST',{map_id:deliveryMapId,point_id:$('singleGoalPoint').value});
  showNotice(r.message||'Đã gửi mục tiêu.',true);
}));
$('cancelSingleGoal').onclick=task(()=>deliveryAction('cancel',async()=>{
  const r=await api('/api/navigation/cancel','POST',{});
  const confirmed=r.success===true&&r.confirmed===true;
  showNotice(r.message||(confirmed?'Nav2 đã kết thúc mục tiêu.':'Đã gửi yêu cầu hủy mục tiêu. Đang chờ xác nhận.'),confirmed);
}));
$('refreshMissionHistory').onclick=task(refreshMissionHistory);
setInterval(()=>{
  if(!loggedIn)return;
  if(Date.now()-deliveryLastStatus>=3500)renderDelivery();
  if(currentTab!=='delivery')return;
  if(deliveryMapId)refreshDeliveryMapMatch().catch(()=>{});
  if(Date.now()-deliveryHistoryAt>5000)refreshMissionHistory();
},1000);
renderDelivery();
if(loggedIn)enterDelivery().catch(err=>showNotice(err.message));

$('logoutMobile').onclick=()=>$('logoutButton').click();
