// Matching the Spinal_tester v1 wire layout. Binary frames start with NCK1.
import { NODE_IDS } from "./protocol.js";

const magic = Uint8Array.of(78, 67, 75, 49);
const wrap = 2 ** 32;
export function crc16(bytes) {
  let crc = 0xffff;
  for (const byte of bytes) {
    crc ^= byte << 8;
    for (let i = 0; i < 8; i++) crc = ((crc << 1) ^ (crc & 0x8000 ? 0x1021 : 0)) & 0xffff;
  }
  return crc;
}

export class WireInput {
  constructor({ onLine, onFrame, onBoundary, onIssue }) {
    Object.assign(this, { onLine, onFrame, onBoundary, onIssue });
    this.pending = new Uint8Array(); this.format = ""; this.session = null;
    this.decoder = new TextDecoder();
  }

  push(bytes) {
    const joined = new Uint8Array(this.pending.length + bytes.length);
    joined.set(this.pending); joined.set(bytes, this.pending.length);
    let pos = 0;
    while (pos < joined.length) {
      const left = joined.subarray(pos);
      if (this.format !== "text" && left.length >= 4 && magic.every((b, i) => left[i] === b)) {
        if (left.length < 8) break;
        const view = new DataView(left.buffer, left.byteOffset, left.length);
        const size = view.getUint16(6,true), total = 34 + size;
        if (left[4] !== 1 || size > 256) { this.onIssue("D0 帧头无效"); pos++; continue; }
        if (left.length < total) break;
        if (crc16(left.subarray(4,total-2)) !== view.getUint16(total-2,true)) {
          this.onIssue("D0 CRC 错误"); pos++; continue;
        }
        this.format = "d0"; this.packet(view,left[5],size); pos += total;
      } else if (this.format === "d0") {
        if (left.length < 4) break;
        pos++;
      } else {
        const nl = left.indexOf(10);
        const mark = left.findIndex((b,i) => b === magic[0] && i+4 <= left.length && magic.every((m,k)=>left[i+k]===m));
        if (mark > 0 && (nl < 0 || mark < nl)) { pos += mark; continue; }
        if (nl < 0) {
          if (left.length > 16384) { this.onIssue("忽略超长输入"); pos += left.length-3; }
          break;
        }
        const line = this.decoder.decode(left.subarray(0,nl)).trim();
        if (line && (/^(?:\{|IMU,|\d+,)/i.test(line) || this.format === "text")) {
          this.format = "text"; this.onLine(line);
        }
        pos += nl+1;
      }
    }
    this.pending = joined.slice(pos);
  }

  packet(v,type,size) {
    const u32 = i=>v.getUint32(i,true);
    const id = [u32(12),u32(16),u32(20)].join("/");
    const emit = Number(v.getBigUint64(24,true));
    if (!Number.isSafeInteger(emit)) { this.onIssue("D0 时间越界"); return; }
    if (type===1 && size===72) {
      if (v.getUint8(44)!==1 || v.getUint8(45)!==3 || v.getUint8(46)!==7 || !(u32(100)&64)) {
        this.session=null; this.onIssue("需要三节点硬件 RAW_FRAME 会话"); return;
      }
      if (this.session!==id || emit<this.lastEmit) { this.onBoundary(); this.seq=null; }
      this.session=id; this.period=u32(48); this.maxAge=u32(52); this.lastEmit=emit; return;
    }
    if (type!==6 || size!==120 || this.session!==id || emit<this.lastEmit) return;
    const sequence=u32(32);
    if (this.seq!=null && ((sequence-this.seq)>>>0) >= 0x80000000) return;
    if (sequence===this.seq || u32(36)!==this.period) return;
    this.lastEmit=emit; this.seq=sequence;
    const samples=[], invalid=[];
    for(let i=0;i<3;i++) {
      const o=44+36*i, node=NODE_IDS[i], flags=v.getUint16(o+34,true), cal=v.getUint8(o+33);
      const start=u32(o), end=u32(o+4), mid=u32(o+8);
      const span=(end-start)>>>0, age=((emit%wrap)-mid)>>>0;
      const raw=k=>v.getInt16(o+k,true);
      const q=[12,14,16,18].map(k=>raw(k)/16384), norm=Math.hypot(...q);
      if (!(v.getUint8(40)&(1<<i)) || !(flags&1) || flags&14 || (cal>>6)<3 || ((cal>>2)&3)<3 ||
          Math.abs(norm-1)>0.001 || age>emit || age>this.maxAge || span>this.maxAge ||
          ((start+Math.floor(span/2))>>>0)!==mid) { invalid.push(node); continue; }
      samples.push({node,timestampUs:emit-age,q:{w:q[0]/norm,x:q[1]/norm,y:q[2]/norm,z:q[3]/norm},
        acceleration:[20,22,24].map(raw),gyroscope:[26,28,30].map(raw),calibration:cal>>6,flags});
    }
    this.onFrame({sequence,samples,invalid,protocol:"D0 RAW_FRAME"},`D0 · ${id} · 帧 ${sequence} · 无效节点 ${invalid.join("、")||"无"}`);
  }
}
