/* Shared exhibit geometry. MIT; SH2ZAM Contributors. */
(function(root){
const TAU=Math.PI*2;
function makeMesh(kind, level) {
    const vertices = [], faces = [];
    const vertex = (x,y,z) => { vertices.push(x,y,z); return vertices.length/3-1; };
    const tri = (a,b,c,color=0) => faces.push({indices:[a,b,c],color});
    const quad = (a,b,c,d,color=0) => { tri(a,b,c,color); tri(a,c,d,color); };
    if(kind === 'saturn') {
      const rows=8+level*3, cols=16+level*8;
      for(let j=0;j<=rows;j++) for(let i=0;i<cols;i++) {
        const p=j*Math.PI/rows, t=i*TAU/cols;
        vertex(1.38*Math.sin(p)*Math.cos(t),1.38*Math.cos(p),1.38*Math.sin(p)*Math.sin(t));
      }
      for(let j=0;j<rows;j++) for(let i=0;i<cols;i++) {
        const a=j*cols+i,b=j*cols+(i+1)%cols;
        quad(a,b,b+cols,a+cols,j%4===0?1:0);
      }
      const bands=[[1.82,2.05],[2.12,2.48],[2.55,2.7]];
      for(let band=0;band<bands.length;band++) {
        const offset=vertices.length/3, steps=cols*2;
        for(let i=0;i<steps;i++) for(const r of bands[band]) {
          const t=i*TAU/steps; vertex(r*Math.cos(t),r*Math.sin(t)*0.36,r*Math.sin(t)*0.933);
        }
        for(let i=0;i<steps;i++) {
          const a=offset+i*2,b=offset+((i+1)%steps)*2;
          quad(a,b,b+1,a+1,band===1?2:1);
        }
      }
    } else if(kind === 'cubes') {
      const n=level+2, step=4.1/n, r=step*0.29;
      for(let x=0;x<n;x++) for(let y=0;y<n;y++) for(let z=0;z<n;z++) {
        const o=vertices.length/3,c=[(x-(n-1)/2)*step,(y-(n-1)/2)*step,(z-(n-1)/2)*step];
        for(const v of [[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]]) vertex(c[0]+r*v[0],c[1]+r*v[1],c[2]+r*v[2]);
        for(const f of [[0,3,2,1],[4,5,6,7],[0,1,5,4],[2,3,7,6],[0,4,7,3],[1,2,6,5]]) quad(...f.map(i=>o+i),(x+y+z)%3);
      }
    } else {
      const rows=48+level*24, cols=6+level*2;
      const center=t=>[(1.65+0.52*Math.cos(3*t))*Math.cos(2*t),(1.65+0.52*Math.cos(3*t))*Math.sin(2*t),0.52*Math.sin(3*t)];
      for(let i=0;i<rows;i++) {
        const t=i*TAU/rows,c=center(t),a=center(t+0.001),v=a.map((x,j)=>x-c[j]);
        const length=Math.hypot(...v); for(let j=0;j<3;j++) v[j]/=length;
        let normal=[-v[1],v[0],0], nl=Math.hypot(...normal);
        if(nl<0.001) { normal=[1,0,0]; nl=1; }
        normal=normal.map(x=>x/nl);
        const b=[v[1]*normal[2]-v[2]*normal[1],v[2]*normal[0]-v[0]*normal[2],v[0]*normal[1]-v[1]*normal[0]];
        for(let j=0;j<cols;j++) { const p=j*TAU/cols; vertex(...c.map((x,k)=>x+0.25*(Math.cos(p)*normal[k]+Math.sin(p)*b[k]))); }
      }
      for(let i=0;i<rows;i++) for(let j=0;j<cols;j++) quad(i*cols+j,((i+1)%rows)*cols+j,((i+1)%rows)*cols+(j+1)%cols,i*cols+(j+1)%cols,Math.floor(i/8)%3);
    }
    return {vertices,faces};
  }
const api={makeMesh};
if(typeof module==='object'&&module.exports)module.exports=api;else root.SH2ZAM_MESHES=api;
})(typeof globalThis==='object'?globalThis:this);
