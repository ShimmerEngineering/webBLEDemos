// --- Shimmer3R over Web Bluetooth: ACK-first Command Flow (v5)
// Adds explicit support for DATA marker (0x00) and **forces 24-bit timestamp** if requested.
// Your note: "timestamp is u24, so layout is: 0x00  [u24 timestamp]  <channel samples...>"
// This build defaults to u24, and you can also pass { timestampFmt: 'u24' } in the constructor.

const OPCODES = { DATA: 0x00, INQUIRY_CMD: 0x01, INQUIRY_RSP: 0x02, START_STREAM: 0x07, ACK: 0xFF, SAMPLING_RATE: 0x05, SET_SENSORS_CMD: 0x08, SET_INTERNAL_EXP_POWER_ENABLE_CMD: 0x5E};

const DEFAULTS = {
  SERVICE_UUID: '65333333-a115-11e2-9e9a-0800200ca100',
  CHAR_RX_UUID: '65333333-a115-11e2-9e9a-0800200ca102',
  CHAR_TX_UUID: '65333333-a115-11e2-9e9a-0800200ca101',
};

const TIMESTAMP_FIELD = {
  u16: { name: 'TIMESTAMP', fmt: 'u16', endian: 'le', sizeBytes: 2 },
  u24: { name: 'TIMESTAMP', fmt: 'u24', endian: 'le', sizeBytes: 3 },
};

// Minimal ObjectCluster analogue
export class ObjectCluster {
  constructor(deviceId){ this.deviceId=deviceId; this.fields=[]; this.raw=null; }
  add(n,v,u=null){ this.fields.push({name:n,value:v,unit:u}); }
  get(n){ return this.fields.find(f=>f.name===n)||null; }
}

// Example subset (extend as needed)
const CHANNEL_FORMATS = {
  0x00: { name: 'ACCEL_X', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x01: { name: 'ACCEL_Y', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x02: { name: 'ACCEL_Z', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x0A: { name: 'GYRO_X', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x0B: { name: 'GYRO_Y', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x0C: { name: 'GYRO_Z', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x27: { name: 'ECG_LA_RA', fmt: 'u12', endian: 'le', sizeBytes: 0 },
  0x28: { name: 'ECG_LL_RA', fmt: 'u12', endian: 'le', sizeBytes: 0 },
  0x12: { name: 'PPG', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x23: { name: 'VSENSE', fmt: 'i16', endian: 'le', sizeBytes: 2 },
};

export class Shimmer3RClient {
  constructor(opts={}){
    this.serviceUUID = opts.serviceUUID || DEFAULTS.SERVICE_UUID;
    this.rxUUID = opts.rxUUID || DEFAULTS.CHAR_RX_UUID;
    this.txUUID = opts.txUUID || DEFAULTS.CHAR_TX_UUID;

    this.device=null; this.server=null; this.rx=null; this.tx=null;
    this._rxBuf=new Uint8Array(0);
    this._temps=new Set();

    this.schema=null; // { timestampFmt, fields[], frameBytes, hasPacked12 }
    this.debug = opts.debug ?? true;

    // Force a specific timestamp width; defaults to 'u24' per your device.
    this.forceTimestampFmt = opts.timestampFmt || 'u24';

    // Stash for ACK+RSP in same notify
    this._lastAckRemainder = null; // Uint8Array | null
  }

  _log(...a){ if(this.debug) console.log('[Shimmer3R]', ...a); }
  _emitStatus(msg){ this._log(msg); this.onStatus?.(msg); }

  async connect(){
    this._emitStatus('Requesting Bluetooth device…');
    this.device = await navigator.bluetooth.requestDevice({ filters:[{services:[this.serviceUUID]}], optionalServices:[this.serviceUUID] });
    this._emitStatus(`Selected: ${this.device.name||'Shimmer3R'}`);
    this.server = await this.device.gatt.connect();
    this._emitStatus('GATT connected');
    const svc = await this.server.getPrimaryService(this.serviceUUID);
    this.rx = await svc.getCharacteristic(this.rxUUID);
    this.tx = await svc.getCharacteristic(this.txUUID);
    this._emitStatus('RX/TX obtained');
    await this.tx.startNotifications();
    this.tx.addEventListener('characteristicvaluechanged', this._handleNotify);
    this._emitStatus('Notifications started');
  }

  async disconnect(){
    try{ if(this.tx){ try{await this.tx.stopNotifications();}catch{} this.tx.removeEventListener('characteristicvaluechanged', this._handleNotify);} if(this.device?.gatt?.connected) this.device.gatt.disconnect(); }
    finally{ this.device=this.server=this.rx=this.tx=null; this._rxBuf=new Uint8Array(0); this.schema=null; this._emitStatus('Disconnected'); }
  }

_handleNotify = (evt) => {
  const chunk = new Uint8Array(evt.target.value.buffer);
  this._log('Notify len=', chunk.length, 'data=', chunk);

  // ACK (may include response remainder)
  if (chunk.length >= 1 && chunk[0] === OPCODES.ACK) {
    this._log('ACK detected at start of notify');
    const remainder = chunk.slice(1);
    this._lastAckRemainder = remainder.length ? remainder : null;
    this._emitTemp(new Uint8Array([OPCODES.ACK]));          // wake ACK waiters
    if (this._lastAckRemainder) {                           // forward remainder
      this._log('Forwarding remainder after ACK', this._lastAckRemainder);
      this._emitTemp(this._lastAckRemainder);
    }
    if (this._lastAckRemainder && this.schema) {            // parse if stream bytes
      this._rxBuf = concatU8(this._rxBuf, this._lastAckRemainder);
    }
    return; // IMPORTANT: stop here for ACK path
  }

  // DATA packet(s) — DO NOT strip; parser expects per-frame 0x00
  if (chunk.length >= 1 && chunk[0] === OPCODES.DATA) {
    if (this.forceTimestampFmt === 'u24' && chunk.length >= 4) {
      const tsPreview = u24le(chunk, 1);
      this._log(`DATA notify → len=${chunk.length} (ts24 preview=${tsPreview})`);
    } else {
      this._log(`DATA notify → len=${chunk.length}`);
    }
    this._rxBuf = concatU8(this._rxBuf, chunk);  // keep 0x00 preambles
  } else {
    // Plain response or raw bytes
    this._emitTemp(chunk);
    this._rxBuf = concatU8(this._rxBuf, chunk);
  }

  // Try parsing if we have a schema
  if (this.schema) {
    try { this._parseBySchema(); }
    catch (e) { this._log('parseBySchema error:', e); }
  }
};

/**
 * Control the internal expansion power (enable/disable)
 * C# equivalent:
 *   WriteBytes([0x5E, expPower], 0, 2)
 * @param {number} expPower - 0 = disable, 1 = enable
 */
async setInternalExpPower(expPower) {
  if (!Number.isInteger(expPower) || expPower < 0 || expPower > 1) {
    throw new Error('expPower must be 0 (off) or 1 (on)');
  }
  if (!this.rx) throw new Error('Not connected (RX missing)');

  const cmd = new Uint8Array([
    OPCODES.SET_INTERNAL_EXP_POWER_ENABLE_CMD,
    expPower & 0xFF,
  ]);

  this._emitStatus(
    `SET_INTERNAL_EXP_POWER_ENABLE_CMD → ${expPower ? 'ON' : 'OFF'} waiting for ACK…`
  );

  await this._write(cmd);
  const ackRemainder = await this._waitForAck(1000);

  this._emitStatus(
    `Expansion power ${expPower ? 'enabled' : 'disabled'} (ACK received).`
  );
  return { expPower, ackRemainder };
}


/**
 * Enable sensors via a 24-bit bitmask.
 * Flow: write → wait for ACK.
 * @param {number} sensors - 24-bit bitmask of sensors to enable (0–0xFFFFFF).
 */
async setSensors(sensors) {
  if (!Number.isFinite(sensors)) {
    throw new Error('Sensors must be a finite number');
  }
  if (!this.rx) throw new Error('Not connected (RX missing)');

  // Coerce to uint32 then clamp to 24 bits
  sensors = (sensors >>> 0) & 0xFFFFFF;

  const b1 = sensors & 0xFF;
  const b2 = (sensors >>> 8) & 0xFF;   // unsigned shift
  const b3 = (sensors >>> 16) & 0xFF;  // unsigned shift
  const cmd = new Uint8Array([OPCODES.SET_SENSORS_CMD, b1, b2, b3]);

  this._emitStatus(
    `SET_SENSORS_CMD → bitmask=0x${sensors.toString(16).toUpperCase().padStart(6, '0')} waiting for ACK…`
  );

  await this._write(cmd);
  const ackRemainder = await this._waitForAck(1000); // was 1000ms

  this._emitStatus(
    `Sensors ACK received. Bitmask 0x${sensors.toString(16).toUpperCase().padStart(6, '0')} applied.`
  );
  return { sensors, ackRemainder };
}



  /**
   * Set device sampling rate in Hz.
   * Firmware expects a 16-bit divisor: divisor = floor(32768 / rateHz), little-endian.
   * Flow: write [0x05, LSB, MSB] → wait for ACK (0xFF) → return applied rate info.
   */
  async setSamplingRate(rateHz) {
    if (!Number.isFinite(rateHz) || rateHz <= 0) {
      throw new Error('Sampling rate must be a positive number (Hz)');
    }
    if (!this.rx) throw new Error('Not connected (RX missing)');

    // Match C#: (int)(32768 / rate)
    let divisor = Math.floor(32768 / rateHz);
    // keep within 16-bit 1..65535
    if (divisor < 1) divisor = 1;
    if (divisor > 0xFFFF) divisor = 0xFFFF;

    const lsb = divisor & 0xFF;
    const msb = (divisor >> 8) & 0xFF;

    const cmd = new Uint8Array([OPCODES.SAMPLING_RATE, lsb, msb]);

    this._emitStatus(`Set sampling rate → ${rateHz.toFixed(3)} Hz (divisor=${divisor}) — waiting for ACK…`);
    await this._write(cmd);

    // Your existing ACK waiter resolves when it sees 0xFF; it also stashes any remainder.
    const ackRemainder = await this._waitForAck(1000);

    // Compute the *applied* Hz from what we actually sent
    const appliedHz = 32768 / divisor;

    // (Optional) if firmware echoes status in ackRemainder, you can parse it here.

    // Update any cached sampling info from Inquiry (purely informational)
    this.lastSamplingDivisor = divisor;
    this.lastSamplingHz = appliedHz;

    this._emitStatus(`Sampling rate ACKed. Applied ≈ ${appliedHz.toFixed(3)} Hz`);
    return { requestedHz: rateHz, appliedHz, divisor, ackRemainder };
  }

  // ---- Commands (ACK then response) ----
  async inquiry(){
    this._emitStatus('INQUIRY_CMD → waiting for ACK then RSP…');
    await this._write(new Uint8Array([OPCODES.INQUIRY_CMD]));
    const remainder = await this._waitForAck(1000); // may be null or a Uint8Array
    if (remainder && remainder[0] === OPCODES.INQUIRY_RSP){
      this._log('Using post-ACK remainder as response');
      const info = this._interpretInquiryResponseShimmer3R(remainder);
      this.onInquiry?.(info); return info;
    }
    const rsp = await this._waitForResponse(OPCODES.INQUIRY_RSP, 1500);
    this._emitStatus(`Inquiry RSP (${rsp.length} bytes)`);
    const info = this._interpretInquiryResponseShimmer3R(rsp);
    this.onInquiry?.(info); return info;
  }

  async startStreaming(){
    if (!this.schema) this._emitStatus('Starting stream without schema (not recommended).');
    this._emitStatus('START_STREAM → waiting for ACK…');
    await this._write(new Uint8Array([OPCODES.START_STREAM]));
    const remainder = await this._waitForAck(1000);
    if (remainder && remainder.length){ this._log('Unexpected data after START_STREAM ACK (appending to buffer):', remainder); this._rxBuf = concatU8(this._rxBuf, remainder); }
    this._emitStatus('START_STREAM ACK received; frames should follow');
  }

  // ---- Inquiry decode → schema ----
  _interpretInquiryResponseShimmer3R(u8){
    let base = 0; if (u8[0]===OPCODES.INQUIRY_RSP && u8.length>=2) base=1;

    const adcRaw = u16le(u8, base+0);
    const samplingHz = 32768/(adcRaw||1);

    const numCh = u8[base+9] ?? 0;
    const bufSize = u8[base+10] ?? 0;
    const chStart = base+11; const chEnd = chStart + numCh; const channelIds = [...u8.slice(chStart, chEnd)];

    // Build schema — **force u24 if configured**
    const guessed = this._guessTimestampFmt(u8, base);
    const tsFmt = this.forceTimestampFmt || guessed;
    const schema = this._buildSchema(channelIds, tsFmt);
    this.schema = schema;

    this._log(`Schema built: timestampFmt=${schema.timestampFmt}, fields=${schema.fields.length}`);

    return { opcode:u8[0], adcRaw, samplingHz, numChannels:numCh, bufferSize:bufSize, channelIds, schema, bytes:u8.slice(0) };
  }
  _guessTimestampFmt(u8, base){ return (u8.length - base) > 20 ? 'u24' : 'u16'; }
  _buildSchema(channelIds, timestampFmt='u24'){
    const fields=[]; const ts = timestampFmt==='u24'?TIMESTAMP_FIELD.u24:TIMESTAMP_FIELD.u16; let frameBytes=ts.sizeBytes; let hasPacked12=false;
    for (const id of channelIds){ const fmt=CHANNEL_FORMATS[id]; if(!fmt){ fields.push({id,name:`CH_${hex2(id)}`,fmt:'i16',endian:'le',sizeBytes:2}); frameBytes+=2; continue; } fields.push({id,...fmt}); if(fmt.fmt==='u12'||fmt.fmt==='i12') hasPacked12=true; else frameBytes+=(fmt.sizeBytes||0); }
    return {
	  timestampFmt,
	  fields,
	  frameBytes,
	  hasPacked12,
	  dataPreambleByte: 0x00   // <-- every frame starts with 0x00
	};
  }

  // ---- Streaming parser (unchanged logic, logs included) ----
  _parseBySchema() {
    const sch = this.schema;
    if (!sch) { this._log('parse: no schema set; skipping'); return; }

    const buf = this._rxBuf;
    let off = 0;
    const needTs = sch.timestampFmt === 'u16' ? 2 : 3;
    let frames = 0;

    this._log(`parse: start — bufferLen=${buf.length}, tsBytes=${needTs}, fields=${sch.fields?.length ?? 0}`);

    while (true) {
	  const remaining = buf.length - off;
	  // We need at least preamble(1) + timestamp bytes to proceed
	  const preBytes = sch.dataPreambleByte != null ? 1 : 0;
	  const needTs = sch.timestampFmt === 'u16' ? 2 : 3;
	  if (remaining < preBytes + needTs) {
		this._log(`parse: not enough bytes for preamble+timestamp: remaining=${remaining}, need=${preBytes + needTs}`);
		break;
	  }

	  let cursor = off;

	  // --- NEW: per-frame preamble ---
	  if (sch.dataPreambleByte != null) {
		if (buf[cursor] !== sch.dataPreambleByte) {
		  // Soft resync: scan ahead for the next 0x00 that leaves enough room for a whole frame
		  const next = buf.indexOf(sch.dataPreambleByte, cursor + 1);
		  if (next === -1 || (buf.length - next) < (1 + needTs)) {
			this._log(`parse: preamble missing; waiting for more (remain=${remaining})`);
			break;
		  }
		  this._log(`parse: resync → skipping ${next - cursor} stray byte(s)`);
		  off = next;      // drop stray bytes
		  continue;        // try again at the found preamble
		}
		cursor += 1; // consume 0x00 preamble
	  }
	  // -------------------------------

	  const oc = new ObjectCluster(this.device?.name || 'Shimmer3R');

	  // Timestamp
	  let ts;
	  if (sch.timestampFmt === 'u16') { ts = u16le(buf, cursor); cursor += 2; }
	  else { ts = u24le(buf, cursor); cursor += 3; }
	  oc.add('TIMESTAMP', ts);

      // For packed 12-bit decoding
      let twelveBuf = 0, twelveBits = 0;

      let ok = true;
      for (const f of sch.fields) {
        const { fmt, endian } = f;
        const before = cursor;

        const need = (fmt === 'u8'  || fmt === 'i8')  ? 1 :
                     (fmt === 'u16' || fmt === 'i16') ? 2 :
                     (fmt === 'u24' || fmt === 'i24') ? 3 :
                     (fmt === 'u12' || fmt === 'i12') ? null : 2; // fallback 2

        if (need !== null && (buf.length - cursor) < need) {
          this._log(`parse: incomplete field ${f.name} fmt=${fmt} need=${need} have=${buf.length - cursor}`);
          ok = false; break;
        }

        let v;
        if (fmt === 'u8' || fmt === 'i8') {
          v = buf[cursor++];
          if (fmt === 'i8') v = (v << 24) >> 24;
        } else if (fmt === 'u16' || fmt === 'i16') {
          v = (endian === 'le') ? u16le(buf, cursor) : u16be(buf, cursor);
          cursor += 2;
          if (fmt === 'i16') v = sign16(v);
        } else if (fmt === 'u24' || fmt === 'i24') {
          v = (endian === 'le') ? u24le(buf, cursor) : u24be(buf, cursor);
          cursor += 3;
          if (fmt === 'i24') v = sign24(v);
        } else if (fmt === 'u12' || fmt === 'i12') {
          while (twelveBits < 12) {
            if ((buf.length - cursor) < 1) { this._log('parse: need more for 12-bit packed'); ok = false; break; }
            twelveBuf |= (buf[cursor++] << twelveBits);
            twelveBits += 8;
          }
          if (!ok) break;
          v = twelveBuf & 0xFFF;
          twelveBuf >>>= 12;
          twelveBits -= 12;
          if (fmt === 'i12' && (v & 0x800)) v |= 0xFFFFF000; // sign-extend
        } else {
          v = u16le(buf, cursor); cursor += 2;
        }

        if (frames < 3) this._log(`parse: field ${f.name} fmt=${fmt} bytes=${cursor - before} value=${v}`);
        oc.add(f.name, v);
      }

      if (!ok) break; // wait for more data

      oc.raw = buf.slice(off, cursor);
      frames++;
      if (frames <= 3 || frames % 50 === 0) this._log(`parse: frame#${frames} bytes=${cursor - off} ts=${ts}`);
      this.onStreamFrame?.(oc);
      off = cursor;
    }

    this._log(`parse: consumed=${off} leftover=${buf.length - off}`);
    this._rxBuf = buf.slice(off);
  }

  // ---- Low-level helpers ----
  async _write(u8){ if(!this.rx) throw new Error('Not connected (RX missing)'); this._log('Write', u8); await this.rx.writeValue(u8); }

  _waitForAck(timeoutMs=1000){
    return new Promise((resolve, reject)=>{
      const t = setTimeout(()=>{ this.offTemp(handler); reject(new Error('ACK timeout')); }, timeoutMs);
      const handler = (chunk)=>{
        if (!chunk||chunk.length===0) return;
        if (chunk.length===1 && chunk[0]===OPCODES.ACK){
          clearTimeout(t); this.offTemp(handler);
          const rem = this._lastAckRemainder; this._lastAckRemainder=null;
          this._log('ACK observed', rem?`(+ remainder len ${rem.length})`:'');
          resolve(rem||null);
        }
      };
      this.onTemp(handler);
    });
  }

  _waitForResponse(expectedOpcode, timeoutMs=1500){
    if (this._lastAckRemainder && this._lastAckRemainder[0]===expectedOpcode){
      const rem=this._lastAckRemainder; this._lastAckRemainder=null; this._log('Using stashed remainder for response');
      return Promise.resolve(rem);
    }
    return new Promise((resolve, reject)=>{
      const t = setTimeout(()=>{ this.offTemp(handler); reject(new Error('Response timeout')); }, timeoutMs);
      const handler = (chunk)=>{
        if (!chunk||chunk.length===0) return; if (chunk.length===1 && chunk[0]===OPCODES.ACK) return;
        if (chunk[0]===expectedOpcode){ clearTimeout(t); this.offTemp(handler); resolve(chunk); }
      };
      this.onTemp(handler);
    });
  }

  onTemp(fn){ this._temps.add(fn); }
  offTemp(fn){ this._temps.delete(fn); }
  _emitTemp(buf){ this._temps.forEach(fn=>{ try{ fn(buf);}catch(e){ this._log('temp handler error', e);} }); }
}

// ---- Byte helpers ----
function concatU8(a,b){ const o=new Uint8Array(a.length+b.length); o.set(a); o.set(b,a.length); return o; }
function u16le(b,o){ return b[o] | (b[o+1]<<8); }
function u16be(b,o){ return (b[o]<<8) | b[o+1]; }
function u24le(b,o){ return b[o] | (b[o+1]<<8) | (b[o+2]<<16); }
function u24be(b,o){ return (b[o]<<16) | (b[o+1]<<8) | b[o+2]; }
function sign16(v){ return (v & 0x8000) ? (v | 0xFFFF0000) : v; }
function sign24(v){ return (v & 0x800000) ? (v | 0xFF000000) : v; }
function hex2(v){ return v.toString(16).padStart(2,'0').toUpperCase(); }
