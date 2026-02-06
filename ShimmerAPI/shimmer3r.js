// --- Shimmer3R over Web Bluetooth: ACK-first Command Flow (v5)
// Adds explicit support for DATA marker (0x00) and **forces 24-bit timestamp** if requested.
// Your note: "timestamp is u24, so layout is: 0x00  [u24 timestamp]  <channel samples...>"
// This build defaults to u24, and you can also pass { timestampFmt: 'u24' } in the constructor.

const OPCODES = { DATA: 0x00, INQUIRY_CMD: 0x01, INQUIRY_RSP: 0x02, START_STREAM: 0x07, STOP_STREAM: 0x20, ACK: 0xFF, SAMPLING_RATE: 0x05, SET_SENSORS_CMD: 0x08, SET_GSR_RANGE: 0x21, SET_INTERNAL_EXP_POWER_ENABLE_CMD: 0x5E, START_BT_STEAM_SD_Logging: 0x70, STOP_BT_STEAM_SD_Logging: 0x97};

const DEFAULTS = {
  SERVICE_UUID: '65333333-a115-11e2-9e9a-0800200ca100',
  CHAR_RX_UUID: '65333333-a115-11e2-9e9a-0800200ca102',
  CHAR_TX_UUID: '65333333-a115-11e2-9e9a-0800200ca101',
};

const TIMESTAMP_FIELD = {
  u16: { name: 'TIMESTAMP', fmt: 'u16', endian: 'le', sizeBytes: 2 },
  u24: { name: 'TIMESTAMP', fmt: 'u24', endian: 'le', sizeBytes: 3 },
};
const GSR_NAME = 'GSR';
const GSR_UNCAL_LIMIT_RANGE3 = 683;
const RAW = 'raw';
const CAL = 'cal';

const SHIMMER3_GSR_RESISTANCE_MIN_MAX_KOHMS = [
    [8.0, 63.0],     // Range 0
    [63.0, 220.0],   // Range 1
    [220.0, 680.0],  // Range 2
    [680.0, 4700.0]  // Range 3
];


// --- Sensor bitmap definitions (Shimmer3) ---
export const SensorBitmapShimmer3 = {
  SENSOR_A_ACCEL:        0x000080,
  SENSOR_GYRO:           0x000040,
  SENSOR_MAG:            0x000020,
  SENSOR_GSR:            0x000004,

  SENSOR_VBATT:          0x002000,
  SENSOR_D_ACCEL:        0x001000,
  SENSOR_PRESSURE:       0x040000,
  SENSOR_EXG1_24BIT:     0x000010,
  SENSOR_EXG2_24BIT:     0x000008,
  SENSOR_EXG1_16BIT:     0x100000,
  SENSOR_EXG2_16BIT:     0x080000,
  SENSOR_BRIDGE_AMP:     0x008000,
  SENSOR_ACCEL_ALT:      0x400000,
  SENSOR_MAG_ALT:        0x200000,

  SENSOR_EXT_A0:         0x000002,
  SENSOR_EXT_A1:         0x000001,
  SENSOR_EXT_A2:         0x000800,
  SENSOR_INT_A3:         0x000400,
  SENSOR_INT_A0:         0x000200,
  SENSOR_INT_A1:         0x000100,
  SENSOR_INT_A2:         0x800000
};

// Minimal ObjectCluster analogue
export class ObjectCluster {
  constructor(deviceId){ 
    this.deviceId = deviceId; 
    this.fields   = []; 
    this.raw      = null; 
  }

  /**
   * Add a field as a separate series.
   * @param {string} name   e.g. 'GYRO_X' or 'GYRO_X_RAW'
   * @param {number} value
   * @param {string|null} unit  e.g. 'deg/s', 'g', 'µS', 'counts', 'raw'
   * @param {'raw'|'cal'|null} kind  tag of the series
   */
  add(name, value, unit=null, kind=null){ 
    this.fields.push({ name, value, unit, kind }); 
  }

  /**
   * Get by name, optionally by kind (when there are both RAW and CAL with same name).
   */
  get(name, kind=null){ 
    return this.fields.find(f => f.name === name && (kind === null || f.kind === kind)) || null; 
  }
}


// Example subset (extend as needed)
const CHANNEL_FORMATS = {
  0x00: { name: 'LN_ACCEL_X', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x01: { name: 'LN_ACCEL_Y', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x02: { name: 'LN_ACCEL_Z', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x04: { name: 'WR_ACCEL_X', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x05: { name: 'WR_ACCEL_Y', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x06: { name: 'WR_ACCEL_Z', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x0A: { name: 'GYRO_X', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x0B: { name: 'GYRO_Y', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x0C: { name: 'GYRO_Z', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x07: { name: 'MAG_X', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x08: { name: 'MAG_Y', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x09: { name: 'MAG_Z', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x1D: { name: 'Exg1_Status', fmt: 'u8', endian: 'le', sizeBytes: 1 },
  0x20: { name: 'Exg2_Status', fmt: 'u8', endian: 'le', sizeBytes: 1 },
  0x1E: { name: 'Exg1_CH1_24Bit', fmt: 'i24', endian: 'be', sizeBytes: 3 },
  0x1F: { name: 'Exg1_CH2_24Bit', fmt: 'i24', endian: 'be', sizeBytes: 3 },
  0x21: { name: 'Exg2_CH1_24Bit', fmt: 'i24', endian: 'be', sizeBytes: 3 },
  0x22: { name: 'Exg2_CH2_24Bit', fmt: 'i24', endian: 'be', sizeBytes: 3 },
  0x23: { name: 'Exg1_CH1_16Bit', fmt: 'i16', endian: 'be', sizeBytes: 2 },
  0x24: { name: 'Exg1_CH2_16Bit', fmt: 'i16', endian: 'be', sizeBytes: 2 },
  0x25: { name: 'Exg2_CH1_16Bit', fmt: 'i16', endian: 'be', sizeBytes: 2 },
  0x26: { name: 'Exg2_CH2_16Bit', fmt: 'i16', endian: 'be', sizeBytes: 2 },
  0x12: { name: 'PPG', fmt: 'i16', endian: 'le', sizeBytes: 2 },
  0x1C: { name: GSR_NAME, fmt: 'u16', endian: 'le', sizeBytes: 2 }
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
    this.enabledSensors = 0x000000; // store 24-bit bitmask
	this.ExpPower = 0;	
    // NEW: ACK expectation counter and streaming state
    this._expectingAck = 0;
    this._streaming = false;
    this.samplingRateHz = 0;
	this.LIMIT_MIN_VALID_USIEMENS = 0.03;
    this.gsrRangeSetting = 0;
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
    try{
      if(this.tx){
        try{await this.tx.stopNotifications();}catch{}
        this.tx.removeEventListener('characteristicvaluechanged', this._handleNotify);
      }
      if(this.device?.gatt?.connected) this.device.gatt.disconnect();
    } finally {
      this.device=this.server=this.rx=this.tx=null;
      this._rxBuf=new Uint8Array(0);
      this.schema=null;
      this._streaming=false;
      // Clear cached expansion power state because we're disconnected
      this.ExpPower = 0;
      this._emitStatus('Disconnected');
    }
  }

  _handleNotify = (evt) => {
    const chunk = new Uint8Array(evt.target.value.buffer);
    this._log('Notify len=', chunk.length, 'data=', chunk);

    // 1) ACK handling only if we are actually expecting an ACK
    if (chunk.length >= 1 && chunk[0] === OPCODES.ACK && (this._expectingAck ?? 0) > 0) {
      this._log('ACK detected at start of notify (expected)');
      // consume one outstanding ACK expectation
      this._expectingAck = Math.max(0, (this._expectingAck || 0) - 1);

      // remainder may be control-plane or stream bytes
      const remainder = chunk.slice(1);
      this._lastAckRemainder = remainder.length ? remainder : null;

      // wake ACK waiters
      this._emitTemp(new Uint8Array([OPCODES.ACK]));

      // If streaming, only append remainder to stream if it *clearly* starts with DATA preamble
      if (this._lastAckRemainder) {
        if (this._streaming && this._lastAckRemainder[0] === OPCODES.DATA) {
          this._log('Appending DATA remainder after ACK to stream buffer');
          this._rxBuf = concatU8(this._rxBuf, this._lastAckRemainder);
        } else {
          // forward to control-plane waiters
          this._log('Forwarding non-DATA remainder to control handlers');
          this._emitTemp(this._lastAckRemainder);
        }
        this._lastAckRemainder = null;
      }
      // Don't parse here; let normal flow continue on next notify
      return;
    }

    // 2) During streaming, *all* bytes (including 0xFF heads) are data-plane
    if (this._streaming) {
      this._rxBuf = concatU8(this._rxBuf, chunk);
    } else {
      // Non-streaming → emit to temp listeners (e.g., responses)
      this._emitTemp(chunk);
      // If we already built schema and non-streaming bytes were actually DATA
      // (rare race), keep them as well to avoid losing frames.
      if (chunk.length && chunk[0] === OPCODES.DATA) {
        this._rxBuf = concatU8(this._rxBuf, chunk);
      }
    }

    // 3) Try parsing if we have a schema
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

    const ackRemainder = await this._writeExpectingAck(cmd, 1500);

    this._emitStatus(
      `Expansion power ${expPower ? 'enabled' : 'disabled'} (ACK received).`
    );
	this.ExpPower = expPower;
    // Notify any UI or caller interested in changes
    try { this.onExpPowerChanged?.(expPower); } catch (e) { this._log('onExpPowerChanged handler error', e); }
    return { expPower, ackRemainder };
  }
  
  /**
   * Control the GSR Range
   * @param {number} gsrRange is between 0 and 4. 0 = 8-63kOhm, 1 = 63-220kOhm, 2 = 220-680kOhm, 3 = 680kOhm-4.7MOhm, 4 = Auto range
   */
  async setGSRRange(gsrRange) {
    if (!Number.isInteger(gsrRange) || gsrRange < 0 || gsrRange > 4) {
      throw new Error('gsr range must be 0-4 value');
    }
    if (!this.rx) throw new Error('Not connected (RX missing)');

    const cmd = new Uint8Array([
      OPCODES.SET_GSR_RANGE,
      gsrRange & 0xFF,
    ]);

    this._emitStatus(
      `SET_GSR_RANGE → waiting for ACK…`
    );

    const ackRemainder = await this._writeExpectingAck(cmd, 1500);

    this._emitStatus(
      `SET_GSR_RANGE (ACK received).`
    );
	this.gsrRangeSetting = gsrRange;
    // Notify any UI or caller interested in changes
    return { gsrRange, ackRemainder };
  }

  getInternalExpPower(){
	return this.ExpPower;
  }
	
  getEnabledSensors() {
    return this.enabledSensors;
  }

  /**
   * Enable sensors via a 24-bit bitmask.
   * Flow: write → wait for ACK → automatically perform Inquiry to refresh schema.
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
    const b2 = (sensors >>> 8) & 0xFF;
    const b3 = (sensors >>> 16) & 0xFF;
    const cmd = new Uint8Array([OPCODES.SET_SENSORS_CMD, b1, b2, b3]);

    this._emitStatus(
      `SET_SENSORS_CMD → bitmask=0x${sensors.toString(16).toUpperCase().padStart(6, '0')} waiting for ACK…`
    );

    const ackRemainder = await this._writeExpectingAck(cmd, 1500);

    this._emitStatus(
      `Sensors ACK received. Bitmask 0x${sensors.toString(16).toUpperCase().padStart(6, '0')} applied.`
    );

    // ✅ Automatically trigger inquiry to rebuild schema and detect active sensors
    try {
      this._emitStatus('Performing automatic inquiry to refresh schema…');
      const info = await this.inquiry();
      this.enabledSensors = info.schema.enabledSensors;
      this._emitStatus(`Inquiry complete. Enabled sensors: 0x${this.enabledSensors.toString(16).toUpperCase()}`);
    } catch (err) {
      this._emitStatus(`Inquiry after setSensors failed: ${err.message}`);
    }

    return { sensors, ackRemainder, enabledSensors: this.enabledSensors };
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

    const ackRemainder = await this._writeExpectingAck(cmd, 1500);

    // Compute the *applied* Hz from what we actually sent
    const appliedHz = 32768 / divisor;

    // Update any cached sampling info from Inquiry (purely informational)
    this.samplingRateHz = appliedHz;

    this._emitStatus(`Sampling rate ACKed. Applied ≈ ${this.samplingRateHz.toFixed(3)} Hz`);
    return { requestedHz: rateHz, appliedHz, divisor, ackRemainder };
  }

  // ---- Commands (ACK then response) ----
  async inquiry(){
    this._emitStatus('INQUIRY_CMD → waiting for ACK then RSP…');

    const remainder = await this._writeExpectingAck(new Uint8Array([OPCODES.INQUIRY_CMD]), 1500);
    if (remainder && remainder[0] === OPCODES.INQUIRY_RSP){
      this._log('Using post-ACK remainder as response');
      const info = this._interpretInquiryResponseShimmer3R(remainder);
      this.onInquiry?.(info); return info;
    }
    const rsp = await this._waitForResponse(OPCODES.INQUIRY_RSP, 2000);
    this._emitStatus(`Inquiry RSP (${rsp.length} bytes)`);
    const info = this._interpretInquiryResponseShimmer3R(rsp);
    this.onInquiry?.(info); return info;
  }
  /**
   * Enable EMG (ADS1292R) in 16-bit mode on EXG1 & EXG2.
   * - Powers the internal expansion rail
   * - Writes the provided EXG1/EXG2 config pages
   * - Enables SENSOR_EXG1_16BIT and SENSOR_EXG2_16BIT via setSensors()
   * - Refreshes schema via inquiry()
   */
  async enableEMG16Bit() {
    if (!this.rx) throw new Error('Not connected (RX missing)');

    // 1) Program ADS1292R pages (EXG1 then EXG2)
    //    These are the exact frames you provided.
    let writeEXG1Command = new Uint8Array([0x61, 0x00, 0x00, 0x0A, 0x02, 0xA8, 0x10, 0x69, 0x60, 0x20, 0x00, 0x00, 0x02, 0x03]);
    let writeEXG2Command = new Uint8Array([0x61, 0x01, 0x00, 0x0A, 0x02, 0xA0, 0x10, 0xE1, 0xE1, 0x00, 0x00, 0x00, 0x02, 0x01]);
    const oversamplingRatio = getOversamplingRatioADS1292R(this.samplingRateHz)
    writeEXG1Command[4] = (((writeEXG1Command[4] >> 3) << 3) | oversamplingRatio) & 0xFF;//index 4 is where the 1st byte of the exg array
    writeEXG2Command[4] = (((writeEXG2Command[4] >> 3) << 3) | oversamplingRatio) & 0xFF;//index 4 is where the 1st byte of the exg array   

    // Many firmwares ACK 0x61, but since that can vary, we do a plain write + short sleep.
    await this._write(writeEXG1Command);
    await new Promise(r => setTimeout(r, 200));
    await this._write(writeEXG2Command);
    await new Promise(r => setTimeout(r, 50));

    // 2) Enable EXG1/EXG2 (16-bit) sensors in the 24-bit bitmap and apply
    const targetBits =
      (SensorBitmapShimmer3.SENSOR_EXG1_16BIT |
       SensorBitmapShimmer3.SENSOR_EXG2_16BIT) >>> 0;

    const newMask = ((this.enabledSensors >>> 0) | targetBits) & 0xFFFFFF;
    await this.setSensors(newMask);   // this already does an inquiry() to rebuild schema

    this._emitStatus('EMG 16-bit enabled on EXG1 & EXG2. Schema updated.');
  }
  
    /**
   * Enable EMG (ADS1292R) in 16-bit mode on EXG1 & EXG2.
   * - Powers the internal expansion rail
   * - Writes the provided EXG1/EXG2 config pages
   * - Enables SENSOR_EXG1_16BIT and SENSOR_EXG2_16BIT via setSensors()
   * - Refreshes schema via inquiry()
   */
  async enableEXGTestSignal16Bit() {
    if (!this.rx) throw new Error('Not connected (RX missing)');

    // 1) Program ADS1292R pages (EXG1 then EXG2)
    //    These are the exact frames you provided.
    let writeEXG1Command = new Uint8Array([0x61, 0x00, 0x00, 0x0A, 0x02, 0xAB, 0x10, 0x15, 0x15, 0x00, 0x00, 0x00, 0x02, 0x01]);
    let writeEXG2Command = new Uint8Array([0x61, 0x01, 0x00, 0x0A, 0x02, 0xA3, 0x10, 0x15, 0x15, 0x00, 0x00, 0x00, 0x02, 0x01]);
    const oversamplingRatio = getOversamplingRatioADS1292R(this.samplingRateHz)
    writeEXG1Command[4] = (((writeEXG1Command[4] >> 3) << 3) | oversamplingRatio) & 0xFF;//index 4 is where the 1st byte of the exg array
    writeEXG2Command[4] = (((writeEXG2Command[4] >> 3) << 3) | oversamplingRatio) & 0xFF;//index 4 is where the 1st byte of the exg array    


    // Many firmwares ACK 0x61, but since that can vary, we do a plain write + short sleep.
    await this._write(writeEXG1Command);
    await new Promise(r => setTimeout(r, 200));
    await this._write(writeEXG2Command);
    await new Promise(r => setTimeout(r, 50));

    // 2) Enable EXG1/EXG2 (16-bit) sensors in the 24-bit bitmap and apply
    const targetBits =
      (SensorBitmapShimmer3.SENSOR_EXG1_16BIT |
       SensorBitmapShimmer3.SENSOR_EXG2_16BIT) >>> 0;

    const newMask = ((this.enabledSensors >>> 0) | targetBits) & 0xFFFFFF;
    await this.setSensors(newMask);   // this already does an inquiry() to rebuild schema

    this._emitStatus('EMG 16-bit enabled on EXG1 & EXG2. Schema updated.');
  }
  
    async enableECG16Bit() {
        if (!this.rx) throw new Error('Not connected (RX missing)');

        // 1) Program ADS1292R pages (EXG1 then EXG2)
        //    These are the exact frames you provided.
        let writeEXG1Command = new Uint8Array([0x61, 0x00, 0x00, 0x0A, 0x02, 0xA8, 0x10, 0x40, 0x40, 0x2D, 0x00, 0x00, 0x02, 0x03]);
        let writeEXG2Command = new Uint8Array([0x61, 0x01, 0x00, 0x0A, 0x02, 0xA0, 0x10, 0x40, 0x47, 0x00, 0x00, 0x00, 0x02, 0x01]);
        const oversamplingRatio = getOversamplingRatioADS1292R(this.samplingRateHz)
        writeEXG1Command[4] = (((writeEXG1Command[4] >> 3) << 3) | oversamplingRatio) & 0xFF;//index 4 is where the 1st byte of the exg array
        writeEXG2Command[4] = (((writeEXG2Command[4] >> 3) << 3) | oversamplingRatio) & 0xFF;//index 4 is where the 1st byte of the exg array    


        // Many firmwares ACK 0x61, but since that can vary, we do a plain write + short sleep.
        await this._write(writeEXG1Command);
        await new Promise(r => setTimeout(r, 200));
        await this._write(writeEXG2Command);
        await new Promise(r => setTimeout(r, 50));

        // 2) Enable EXG1/EXG2 (16-bit) sensors in the 24-bit bitmap and apply
        const targetBits =
            (SensorBitmapShimmer3.SENSOR_EXG1_16BIT |
                SensorBitmapShimmer3.SENSOR_EXG2_16BIT) >>> 0;

        const newMask = ((this.enabledSensors >>> 0) | targetBits) & 0xFFFFFF;
        await this.setSensors(newMask);   // this already does an inquiry() to rebuild schema

        this._emitStatus('EMG 16-bit enabled on EXG1 & EXG2. Schema updated.');
    }
  
  async startStreaming(){
    if (!this.schema) this._emitStatus('Starting stream without schema (not recommended).');
    this._emitStatus('START_STREAM → waiting for ACK…');

    const remainder = await this._writeExpectingAck(new Uint8Array([OPCODES.START_STREAM]), 1500);

    // Now streaming
    this._streaming = true;

    if (remainder && remainder.length){
      if (remainder[0] === OPCODES.DATA) {
        this._log('Unexpected DATA after START_STREAM ACK (appending to buffer)');
        this._rxBuf = concatU8(this._rxBuf, remainder);
      } else {
        this._emitTemp(remainder); // non-DATA control-plane
      }
    }
    this._emitStatus('START_STREAM ACK received; frames should follow');
  }

  async stopStreaming() {
    this._emitStatus('STOP_STREAM → sending (no ACK wait)…');

    try {
      await this._write(new Uint8Array([OPCODES.STOP_STREAM]));
      this._emitStatus('STOP_STREAM command sent (skipped ACK wait).');
    } catch (err) {
      this._emitStatus(`STOP_STREAM write failed: ${err.message}`);
    }

    this._streaming = false;         // stop treating notifies as data-plane
    this._rxBuf = new Uint8Array(0); // optional flush
    this._emitStatus('Streaming stopped (ACK skipped for latency tolerance).');
  }
  
  async startStreamingandLogging(){
    if (!this.schema) this._emitStatus('Starting stream without schema (not recommended).');
    this._emitStatus('START_STREAM → waiting for ACK…');

    const remainder = await this._writeExpectingAck(new Uint8Array([OPCODES.START_BT_STEAM_SD_Logging]), 1500);

    // Now streaming
    this._streaming = true;

    if (remainder && remainder.length){
      if (remainder[0] === OPCODES.DATA) {
        this._log('Unexpected DATA after START_STREAM ACK (appending to buffer)');
        this._rxBuf = concatU8(this._rxBuf, remainder);
      } else {
        this._emitTemp(remainder); // non-DATA control-plane
      }
    }
    this._emitStatus('START_STREAM ACK received; frames should follow');
  }

  async stopStreamingandLogging() {
    this._emitStatus('STOP_STREAM → sending (no ACK wait)…');

      try {
          await this._write(new Uint8Array([OPCODES.STOP_BT_STEAM_SD_Logging]));
      this._emitStatus('STOP_STREAM command sent (skipped ACK wait).');
    } catch (err) {
      this._emitStatus(`STOP_STREAM write failed: ${err.message}`);
    }

    this._streaming = false;         // stop treating notifies as data-plane
    this._rxBuf = new Uint8Array(0); // optional flush
    this._emitStatus('Streaming stopped (ACK skipped for latency tolerance).');
  }

  // ---- Inquiry decode → schema ----
  // ✅ Best-practice version of `_interpretInquiryResponseShimmer3R()` and `_buildSchema()`
// This approach mirrors the C# `InterpretDataPacketFormat()` logic more closely.
// It returns a schema object that includes not only field info, but also enabledSensors and packetSize.

  _interpretInquiryResponseShimmer3R(u8) {
    let base = 0;
    if (u8[0] === OPCODES.INQUIRY_RSP && u8.length >= 2) base = 1; //base is 1 because the first index is the response code

      const adcRaw = u16le(u8, base + 0);
      this._log("adc raw: " + adcRaw);
      const samplingRateHz = 32768 / adcRaw;
      this.samplingRateHz = samplingRateHz;
      this._log("sampling rate: " + this.samplingRateHz);
	
	const cfg =
      (BigInt(u8[base+2])       ) |
      (BigInt(u8[base+3]) << 8n) |
      (BigInt(u8[base+4]) << 16n) |
      (BigInt(u8[base+5]) << 24n) |
      (BigInt(u8[base+6]) << 32n) |
      (BigInt(u8[base+7]) << 40n) |
      (BigInt(u8[base+8]) << 48n);

    const internalExpPower = Number((cfg >> 24n) & 0x1n);
	const gsrRange = Number((cfg >> 25n) & 0x7n);
    this.ExpPower = internalExpPower;
	this.gsrRangeSetting = gsrRange;
	const numCh = u8[base + 9] ?? 0;
    const bufSize = u8[base + 10] ?? 0;
    const chStart = base + 11;
    const chEnd = chStart + numCh;
    const channelIds = [...u8.slice(chStart, chEnd)];

    // Build schema — fixed to 24-bit timestamp format
    const tsFmt = 'u24'; // Always use 24-bit timestamp
    const schema = this._buildSchemaFromChannels(channelIds, tsFmt);

    this.schema = schema;
    this._log(
      `Schema built: timestampFmt=${schema.timestampFmt}, fields=${schema.fields.length}, enabledSensors=0x${schema.enabledSensors.toString(16)}`
    );
    this._emitStatus(
      `Expansion power ${this.ExpPower ? 'enabled' : 'disabled'} (ACK received).`
    );
    return {
      opcode: u8[0],
      adcRaw,
      samplingRateHz,
      numChannels: numCh,
      bufferSize: bufSize,
      channelIds,
      schema,
      bytes: u8.slice(0)
    };
  }

  _buildSchemaFromChannels(channelIds, timestampFmt = 'u24') {
    const fields = [];
    const ts = timestampFmt === 'u24' ? TIMESTAMP_FIELD.u24 : TIMESTAMP_FIELD.u16;

    // ✅ Include DATA preamble (0x00) in frame size for boundary check
    let packetSize = 1 + ts.sizeBytes; // 1 = preamble 0x00
    let hasPacked12 = false;
    let enabledSensors = 0;

    for (const id of channelIds) {
      const fmt = CHANNEL_FORMATS[id];
      if (!fmt) {
        fields.push({ id, name: `CH_${hex2(id)}`, fmt: 'i16', endian: 'le', sizeBytes: 2 });
        packetSize += 2;
        continue;
      }

      fields.push({ id, ...fmt });
      packetSize += fmt.sizeBytes || 2;

      if (fmt.fmt === 'u12' || fmt.fmt === 'i12') hasPacked12 = true;

      // Minimal sensor bitmask mapping (extend as needed)
      switch (id) {
        case 0x00: case 0x01: case 0x02:
          enabledSensors |= SensorBitmapShimmer3.SENSOR_A_ACCEL; break;
        case 0x04: case 0x05: case 0x06:
          enabledSensors |= SensorBitmapShimmer3.SENSOR_D_ACCEL; break;
        case 0x07: case 0x08: case 0x09:
          enabledSensors |= SensorBitmapShimmer3.SENSOR_MAG; break;
        case 0x0A: case 0x0B: case 0x0C:
          enabledSensors |= SensorBitmapShimmer3.SENSOR_GYRO; break;
        case 0x12:
          enabledSensors |= SensorBitmapShimmer3.SENSOR_INT_A1; break;
        case 0x1C:
          enabledSensors |= SensorBitmapShimmer3.SENSOR_GSR; break;
        case 0x23: case 0x24:
          enabledSensors |= SensorBitmapShimmer3.SENSOR_EXG1_16BIT; break;
        case 0x25: case 0x26:
          enabledSensors |= SensorBitmapShimmer3.SENSOR_EXG2_16BIT; break;
        case 0x1E: case 0x1F:
          enabledSensors |= SensorBitmapShimmer3.SENSOR_EXG1_24BIT; break;
        case 0x21: case 0x22:
         enabledSensors |= SensorBitmapShimmer3.SENSOR_EXG2_24BIT; break;
		default:
         console.warn(`⚠️ Unmapped channel ID 0x${id.toString(16)} — added as generic i16.`);
      }
    }

    this.enabledSensors = enabledSensors;

    return {
      timestampFmt,
      fields,
      frameBytes: packetSize,          // ✅ inclusive of preamble
      hasPacked12,
      enabledSensors,
      dataPreambleByte: 0x00
    };
  }

  _calibrateData(oc){
    const snapshot = [...oc.fields]; // copy so pushes won't affect iteration
	for (const field of snapshot) {
		if (field.name === GSR_NAME){
			const field = oc.get(GSR_NAME, RAW);
			const gsrraw = field?.value ?? null;
			//console.log(gsrraw);
			let adc12 = (gsrraw & 0x0FFF);
			let currentRange = this.gsrRangeSetting;
			if (currentRange === 4) {
			  currentRange = (gsrraw >> 14) & 0x03; // auto-range bits
            }
            if (currentRange === 3) {
                if (adc12 < GSR_UNCAL_LIMIT_RANGE3) {
                    adc12 = GSR_UNCAL_LIMIT_RANGE3;
                }
            }
            let gsrkOhm = calibrateGsrDataToResistanceFromAmplifierEq(adc12, currentRange);
            gsrkOhm = nudgeGsrResistance(gsrkOhm, this.gsrRangeSetting);
            let gsrConductanceUSiemens = (1.0 / gsrkOhm) * 1000;
            //console.log('uSiemens: ' + gsrConductanceUSiemens + ' ' + this.gsrRangeSetting + ' ' + currentRange);
            oc.add(GSR_NAME, gsrConductanceUSiemens, 'uSiemens', CAL);	
            //oc.add(GSR_NAME, gsr, 'kOhm', CAL);	

		}
	}
  }	  

  _parseBySchema() {
    const sch = this.schema;
    if (!sch) { this._log('parse: no schema set; skipping'); return; }

    const preamble   = sch.dataPreambleByte ?? 0x00;  // 0x00 = DATA marker
    const frameBytes = sch.frameBytes >>> 0;          // includes preamble + ts + fields
    const tsBytes    = sch.timestampFmt === 'u16' ? 2 : 3;
    const TS_MOD     = tsBytes === 3 ? 16777216 : 65536; // 2^24 or 2^16

    let buf = this._rxBuf;
    let frames = 0, drops = 0, anomalies = 0;

    if (this._lastTs === undefined || this._lastTs === null) this._lastTs = 0;

    // Need at least two full frames to assert boundary: [0x00 ...][0x00 ...]
    while (buf.length >= frameBytes * 2) {
      // Quick preamble test at current offset and next-frame offset
      if (buf[0] === preamble && buf[frameBytes] === preamble) {
        // --- Peek timestamps from frame #1 and frame #2 to validate boundary ---
        let ts1, ts2;
        try {
          ts1 = (tsBytes === 2) ? u16le(buf, 1) : u24le(buf, 1);
          ts2 = (tsBytes === 2) ? u16le(buf, frameBytes + 1) : u24le(buf, frameBytes + 1);
        } catch {
          // If peek fails, slide one byte and retry
          buf = buf.subarray(1);
          drops++;
          continue;
        }

        // Monotonic check modulo TS range: dt must be > 0 (allow wrap)
        const dt = ( (ts2 - ts1) % TS_MOD + TS_MOD ) % TS_MOD;
        if (dt === 0) {
          // False boundary (duplicate ts or corruption) → slide one byte
          buf = buf.subarray(1);
          drops++;
          continue;
        }

        // --- Decode *one* frame now that boundary is plausible ---
        const frame = buf.subarray(0, frameBytes);

        try {
          let cursor = 1; // skip preamble

          const oc = new ObjectCluster(this.device?.name || 'Shimmer3R');

          // Timestamp
          let ts;
          if (tsBytes === 2) { ts = u16le(frame, cursor); cursor += 2; }
          else { ts = u24le(frame, cursor); cursor += 3; }
          oc.add('TIMESTAMP', ts, 'ticks', RAW);

          // Fields
          for (const f of sch.fields) {
            if (cursor + f.sizeBytes > frame.length) {
              throw new Error(`short frame: need ${f.sizeBytes} @${cursor}, have ${frame.length}`);
            }
            let v;
            switch (f.fmt) {
			  case 'i16':
				v = f.endian === 'be' ? sign16(u16be(frame, cursor)) : sign16(u16le(frame, cursor));
				break;
			  case 'u16':
				v = f.endian === 'be' ? u16be(frame, cursor) : u16le(frame, cursor);
				break;
     		  case 'i24':
				v = f.endian === 'be' ? sign24(u24be(frame, cursor)) : sign24(u24le(frame, cursor));
				break;
			  case 'u24':
				v = f.endian === 'be' ? u24be(frame, cursor) : u24le(frame, cursor);
				break;				
              case 'u8':  v = frame[cursor]; break;
              default:    v = u16le(frame, cursor); // fallback
            }
            cursor += f.sizeBytes;
            oc.add(f.name, v, null,RAW);
          }

          // Optional: anomaly log against lastTs (purely diagnostic)
          if (this._lastTs) {
            const dLast = ( (ts - this._lastTs) % TS_MOD + TS_MOD ) % TS_MOD;
            if (dLast === 0) {
              anomalies++;
              this._log(`⚠️ Timestamp anomaly#${anomalies}: ts=${ts}, last=${this._lastTs}, Δ=0, frameLen=${frame.length}`);
            }
          }
          this._lastTs = ts;
			
		  this._calibrateData(oc); 	
          
		  // Emit and consume one frame
          this.onStreamFrame?.(oc);
          frames++;
          buf = buf.subarray(frameBytes);
          continue;
        } catch (e) {
          // If field decode failed despite boundary check, drop 1 byte to resync
          this._log('⚠️ frame decode error → sliding 1 byte', e.message);
          buf = buf.subarray(1);
          drops++;
          continue;
        }
      }

      // ❌ Not aligned: drop 1 byte and retry
      buf = buf.subarray(1);
      drops++;
      if (this.debug && drops % 64 === 1) {
        this._log(`resync: dropped ${drops} byte(s) so far; bufLen=${buf.length}`);
      }
    }

    // Keep remainder for next notify (may be < 2 frames or partial)
    this._rxBuf = buf;

    // If we dropped a lot, reset lastTs so future monotonic checks don’t spam
    if (drops && drops % 512 === 0) this._lastTs = 0;

    if (this.debug && (frames || drops)) {
      this._log(`parse: frames=${frames}, drops=${drops}, leftover=${this._rxBuf.length}`);
    }
  }

  // ---- Low-level helpers ----
  async _write(u8){
    if(!this.rx) throw new Error('Not connected (RX missing)');
    this._log('Write', u8);
    await this.rx.writeValue(u8);
  }

  // NEW: helper that increments ACK expectation, writes, and waits
  async _writeExpectingAck(u8, ackTimeoutMs = 1000) {
    this._expectingAck++;
    try {
      await this._write(u8);
      const rem = await this._waitForAck(ackTimeoutMs);
      return rem; // may be null or a Uint8Array (remainder)
    } catch (e) {
      this._expectingAck = Math.max(0, this._expectingAck - 1);
      throw e;
    }
  }

  _waitForAck(timeoutMs=1000){
    return new Promise((resolve, reject)=>{
      const t = setTimeout(()=>{ this.offTemp(handler); reject(new Error('ACK timeout')); }, timeoutMs);
      const handler = (chunk)=>{
        if (!chunk||chunk.length===0) return;

        // pure ACK
        if (chunk.length===1 && chunk[0]===OPCODES.ACK){
          clearTimeout(t); this.offTemp(handler);
          const rem = this._lastAckRemainder; this._lastAckRemainder=null;
          this._log('ACK observed', rem?`(+ remainder len ${rem.length})`:'');
          resolve(rem||null);
          return;
        }

        // concatenated ACK + remainder that got emitted directly to temp listeners
        if (chunk[0] === OPCODES.ACK && chunk.length > 1) {
          clearTimeout(t); this.offTemp(handler);
          const rem = chunk.slice(1);
          this._log('ACK observed (inline) (+ remainder len', rem.length, ')');
          resolve(rem);
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
        if (!chunk||chunk.length===0) return;
        if (chunk.length===1 && chunk[0]===OPCODES.ACK) return;
        if (chunk[0]===expectedOpcode){ clearTimeout(t); this.offTemp(handler); resolve(chunk); }
      };
      this.onTemp(handler);
    });
  }

  onTemp(fn){ this._temps.add(fn); }
  offTemp(fn){ this._temps.delete(fn); }
  _emitTemp(buf){ this._temps.forEach(fn=>{ try{ fn(buf);}catch(e){ this._log('temp handler error', e);} }); }
}

function calibrateGsrDataToResistanceFromAmplifierEq(gsrUncalibratedData, range) {
  const SHIMMER3_REF_KOHMS = [40.2, 287.0, 1000.0, 3300.0];
  const rFeedback = SHIMMER3_REF_KOHMS[range];
  let volts = 0;
  volts = calibrateShimmer3RAdcChannel(gsrUncalibratedData) / 1000.0; // convert mV → V
  // Amplifier equation: Rsource = Rfeedback / ((Vout / 0.5) - 1)
  const rSource = rFeedback / ((volts / 0.5) - 1.0);
  return rSource; // in kΩ
}

function calibrateShimmer3RAdcChannel(unCalData) {
  const offset = 0;
  const vRefP = 3; // reference voltage in volts
  const gain = 1;
  return calibrateU12AdcValue(unCalData, offset, vRefP, gain);
}

function calibrateU12AdcValue(uncalibratedData, offset, vRefP, gain) {
  // Formula: (uncalibrated - offset) * ((vRefP * 1000 / gain) / 4095)
  const calibratedData = (uncalibratedData - offset) * (((vRefP * 1000) / gain) / 4095);
  return calibratedData;
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
function getOversamplingRatioADS1292R(samplingRate) {
  if (!Number.isFinite(samplingRate)) {
    throw new TypeError('samplingRate must be a finite number');
  }
  if (samplingRate < 0) {
    throw new RangeError('samplingRate must be non-negative');
  }

  let oversamplingRatio = 6; // >=4000

  if (samplingRate < 125)        oversamplingRatio = 0;
  else if (samplingRate < 250)   oversamplingRatio = 1;
  else if (samplingRate < 500)   oversamplingRatio = 2;
  else if (samplingRate < 1000)  oversamplingRatio = 3;
  else if (samplingRate < 2000)  oversamplingRatio = 4;
  else if (samplingRate < 4000)  oversamplingRatio = 5;

  return oversamplingRatio;
}
function nudgeDouble(valToNudge, minVal, maxVal) {
    return Math.max(minVal, Math.min(maxVal, valToNudge));
}
function nudgeGsrResistance(gsrResistanceKOhms, gsrRangeSetting) {
    // In your C# code, rangeSetting == 4 means "no nudge" (pass-through)
    if (gsrRangeSetting !== 4) {
        const [minVal, maxVal] = SHIMMER3_GSR_RESISTANCE_MIN_MAX_KOHMS[gsrRangeSetting];
        return nudgeDouble(gsrResistanceKOhms, minVal, maxVal);
    }
    return gsrResistanceKOhms;
}
