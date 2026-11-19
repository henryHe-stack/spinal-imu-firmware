import assert from "node:assert/strict";
import test from "node:test";
import { WireInput, crc16 } from "../js/local-wire.js";

function packet(type,seq,session=1,flags=1) {
  const b=Buffer.alloc(type===1?106:154);
  b.write("NCK1"); b[4]=1; b[5]=type; b.writeUInt16LE(b.length-34,6);
  b.writeUInt32LE(seq,8); b.writeUInt32LE(1,12); b.writeUInt32LE(session,16); b.writeUInt32LE(1,20);
  const t=2**32+500+seq*10000; b.writeBigUInt64LE(BigInt(t),24);
  if(type===1) { b[44]=1;b[45]=3;b[46]=7;b.writeUInt32LE(10000,48);b.writeUInt32LE(30000,52);b.writeUInt32LE(127,100); }
  else {
    b.writeUInt32LE(seq,32); b.writeUInt32LE(10000,36); b[40]=7;
    for(let i=0;i<3;i++) {
      const o=44+36*i,mid=t-2000+500*i;
      b.writeUInt32LE((mid-100)>>>0,o);b.writeUInt32LE((mid+100)>>>0,o+4);b.writeUInt32LE(mid>>>0,o+8);
      b.writeInt16LE(16384,o+12);b.writeInt16LE(981,o+24);b[o+33]=255;b.writeUInt16LE(i===1?flags:1,o+34);
    }
  }
  b.writeUInt16LE(crc16(b.subarray(4,-2)),b.length-2);return b;
}
test("D0 sessions, fragmentation, CRC recovery, stale sessions and quality",()=>{
  const frames=[],issues=[];let boundaries=0;
  const input=new WireInput({onFrame:f=>frames.push(f),onIssue:m=>issues.push(m),onBoundary:()=>boundaries++,onLine:()=>assert.fail()});
  const damaged=packet(6,2);damaged[53]^=1;
  const bytes=Buffer.concat([packet(6,0),packet(1,0),packet(6,1),damaged,packet(6,3,1,3),packet(6,3),packet(1,4,2),packet(6,5,1),packet(6,5,2)]);
  for(let i=0;i<bytes.length;i+=3) input.push(bytes.subarray(i,i+3));
  assert.equal(boundaries,2);assert.equal(issues.length,1);assert.deepEqual(frames.map(f=>f.sequence),[1,3,5]);
  assert.deepEqual(frames[1].invalid,["C4"]);assert.equal(frames[0].samples[0].acceleration[2],981);
  assert.equal(frames[0].samples[0].timestampUs,2**32+8500);
});
test("text CSV and NDJSON line delivery survives arbitrary splits",()=>{
  for(const text of ['IMU,1,C0,10,1,0,0,0,3','{"seq":1,"samples":[]}']) {
    const lines=[],input=new WireInput({onLine:s=>lines.push(s),onIssue:assert.fail});
    for(const byte of Buffer.from(text+"\r\n"))input.push(Uint8Array.of(byte));
    assert.deepEqual(lines,[text]);
  }
});
