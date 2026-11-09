(() => {
'use strict';
const $ = id => document.getElementById(id);
const canvas=$('map'),ctx=canvas.getContext('2d');
let data=null,index=0,playing=false,last=0,accumulator=0,editing=false,draft=[],bounds=null;
const stateNames=['INIT','READY','RUNNING','PAUSED','DEGRADED','STOPPED','FAULT','ESTOP'];
function message(s){$('message').textContent=s;}
function validate(d){
 if(!d||d.format!=='ROS2DriveStudioV1'||!Array.isArray(d.frames)||!Array.isArray(d.route)||!Array.isArray(d.lanes))throw Error('Unsupported recording format');
 if(d.frames.length>100000)throw Error('Recording too large (100,000 frame limit)');
 for(const f of d.frames){if(!f.pose||![f.pose.x,f.pose.y,f.pose.yaw,f.t].every(Number.isFinite))throw Error('Invalid vehicle frame');}
 return d;
}
function load(d){
 data=validate(d);index=0;playing=false;draft=[];bounds=computeBounds();$('scene').textContent=data.scenario;
 $('timeline').max=Math.max(0,data.frames.length-1);$('timeline').value=0;$('play').textContent='▶ Play';
 message('Loaded '+data.frames.length+' frames from C++ simulation.');render();
}
function allPoints(){return [...(data?.route||[]),...(data?.lanes||[]).flatMap(l=>l.points||[]),...(data?.frames||[]).map(f=>f.pose),...draft];}
function computeBounds(){
 const p=allPoints().filter(p=>Number.isFinite(p.x)&&Number.isFinite(p.y));
 if(!p.length)return {cx:0,cy:0,span:70};
 let minX=Infinity,maxX=-Infinity,minY=Infinity,maxY=-Infinity;for(const v of p){minX=Math.min(minX,v.x);maxX=Math.max(maxX,v.x);minY=Math.min(minY,v.y);maxY=Math.max(maxY,v.y);}
 return {cx:(minX+maxX)/2,cy:(minY+maxY)/2,span:Math.max(maxX-minX,maxY-minY,15)*1.2};
}
function transform(){const w=canvas.clientWidth,h=canvas.clientHeight,b=bounds||{cx:0,cy:0,span:70};const s=Math.min(w,h)/b.span;return {s,to:p=>({x:w/2+(p.x-b.cx)*s,y:h/2-(p.y-b.cy)*s}),from:(x,y)=>({x:(x-w/2)/s+b.cx,y:(h/2-y)/s+b.cy})};}
function path(points,color,width=2,dashed=false){
 if(!points?.length)return;const t=transform();ctx.beginPath();points.forEach((p,i)=>{const q=t.to(p);i?ctx.lineTo(q.x,q.y):ctx.moveTo(q.x,q.y);});
 ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dashed?[8,7]:[]);ctx.stroke();ctx.setLineDash([]);
}
function render(){
 const dpr=window.devicePixelRatio||1,w=canvas.clientWidth,h=canvas.clientHeight;
 if(!w||!h)return;
 canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);
 const t=transform(),b=bounds||{cx:0,cy:0,span:70};
 ctx.strokeStyle='#213743';ctx.lineWidth=1;
 for(let x=((w/2-b.cx*t.s)%(10*t.s)+10*t.s)%(10*t.s);x<w;x+=10*t.s){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,h);ctx.stroke();}
 for(let y=((h/2+b.cy*t.s)%(10*t.s)+10*t.s)%(10*t.s);y<h;y+=10*t.s){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke();}
 if(data){
 if($('lanes').checked)for(const lane of data.lanes)path(lane.points,'#385461',9);
 if($('route').checked)path(data.route,'#62d99c',2);
 if($('obstacles').checked)for(const o of data.obstacles||[]){const p=t.to(o.pose);ctx.beginPath();ctx.arc(p.x,p.y,Math.max(4,o.radius*t.s),0,Math.PI*2);ctx.fillStyle='#ba694d';ctx.fill();}
 const f=data.frames[index];
 if(f){
 if($('trajectory').checked)path(f.trajectory,'#f2be64',2,true);
 const p=t.to(f.pose),a=-f.pose.yaw;ctx.save();ctx.translate(p.x,p.y);ctx.rotate(a);ctx.fillStyle='#f2c55b';ctx.fillRect(-12,-7,24,14);ctx.fillStyle='#fff0b4';ctx.fillRect(4,-5,6,10);ctx.restore();
 $('speed').textContent=(f.speed??0).toFixed(2)+' m/s';$('steering').textContent=(f.steering??0).toFixed(3)+' rad';
 $('state').textContent=stateNames[f.state]||String(f.state);$('planning').textContent=f.planning_ok?'OK':'FAILED';
 $('frame').textContent=(index+1)+' / '+data.frames.length;$('clock').textContent=f.t.toFixed(2)+' s';$('timeline').value=index;
 }
 }
 path(draft,'#55bafa',2,true);
 for(const p of draft){const q=t.to(p);ctx.beginPath();ctx.arc(q.x,q.y,4,0,Math.PI*2);ctx.fillStyle='#55bafa';ctx.fill();}
}
function tick(ts){
 if(playing&&data?.frames.length){if(last){accumulator+=(ts-last)*Number($('rate').value)/1000;const dt=data.dt||0.05;let jump=Math.floor(accumulator/dt);if(jump){accumulator-=jump*dt;index=Math.min(index+jump,data.frames.length-1);render();if(index===data.frames.length-1){playing=false;$('play').textContent='▶ Play';}}}}
 last=ts;requestAnimationFrame(tick);
}
$('play').onclick=()=>{if(!data)return;if(index===data.frames.length-1)index=0;playing=!playing;last=0;accumulator=0;$('play').textContent=playing?'Ⅱ Pause':'▶ Play';render();};
$('reset').onclick=()=>{index=0;playing=false;$('play').textContent='▶ Play';render();};
$('timeline').oninput=e=>{index=Number(e.target.value);render();};
$('file').onchange=async e=>{const f=e.target.files[0];if(!f)return;try{load(JSON.parse(await f.text()));}catch(err){message(err.message);}};
$('reload').onclick=async()=>{try{const res=await fetch('recording.json?ts='+Date.now(),{cache:'no-store'});if(!res.ok)throw Error('No recording.json found. Generate it with ros2drive_sim first.');load(await res.json());}catch(err){message(err.message);}};
$('edit').onclick=()=>{editing=!editing;$('edit').textContent='Edit route: '+(editing?'ON':'OFF');};
$('clear').onclick=()=>{draft=[];render();};
canvas.addEventListener('click',e=>{if(!editing)return;const r=canvas.getBoundingClientRect();const p=transform().from(e.clientX-r.left,e.clientY-r.top);draft.push({x:p.x,y:p.y,yaw:0,z:0});render();});
$('export').onclick=()=>{if(draft.length<2){message('Add at least two draft waypoints.');return;}const blob=new Blob([JSON.stringify({format:'ROS2DriveRouteDraftV1',scenario:data?.scenario||null,waypoints:draft},null,2)],{type:'application/json'});const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='route-draft.json';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);message('Draft exported. This does not automatically modify the C++ mission.');};
for(const id of ['lanes','route','trajectory','obstacles'])$(id).onchange=render;
new ResizeObserver(render).observe(canvas);requestAnimationFrame(tick);
$('reload').click();
})();