/**
 * Verisense (Web Bluetooth) – single-file JS class
 * - Connects to Verisense over Nordic UART Service (NUS)
 * - Supports command requests + streaming reassembly
 * - Decodes sensors: GSR (id=1), LIS2DW12 accel (id=2), LSM6DS3 accel/gyro (id=3), PPG (id=4)
 *
 * Notes:
 * - Web Bluetooth requires HTTPS + a user gesture (e.g., button click).
 * - This mirrors the C# streaming shape: [hdr][lenLE16] + payload chunks,
 *   where payload starts with: [sensorId][tick u24][sensorPayload...][optional CRC16].
 */

class TinyEmitter {
  constructor() { this._m = new Map(); }
  on(ev, fn) { if (!this._m.has(ev)) this._m.set(ev, new Set()); this._m.get(ev).add(fn); return () => this.off(ev, fn); }
  off(ev, fn) { const s = this._m.get(ev); if (s) s.delete(fn); }
  emit(ev, data) { const s = this._m.get(ev); if (s) for (const fn of s) fn(data); }
}

// ---------- helpers ----------
function u16le(b0, b1) { return (b1 << 8) | b0; }
function i16le(bytes, off) {
  const v = (bytes[off] | (bytes[off + 1] << 8));
  return (v & 0x8000) ? v - 0x10000 : v;
}
function u24le(bytes, off) { return (bytes[off] | (bytes[off + 1] << 8) | (bytes[off + 2] << 16)) >>> 0; }
function nowMillis() { return Date.now(); }

function crc16_ccitt_false(bytes) {
  // CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, xorOut 0x0000
  let crc = 0xFFFF;
  for (let i = 0; i < bytes.length; i++) {
    crc ^= (bytes[i] << 8);
    for (let b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
      crc &= 0xFFFF;
    }
  }
  return crc & 0xFFFF;
}

// ---------- base sensor (timestamp unwrap + extrapolation) ----------
class SensorBase {
  // matches Sensor.cs
  static CLOCK_FREQ = 32768;                 // ticks/sec
  static TICKS_MAX_VALUE = 60 * 32768;       // 1 minute rollover

  constructor() {
    this.lastTicksUnwrapped = 0;
    this.cycle = 0;
    this.systemOffsetFirstTime = null; // systemMillis - shimmerMillis
    this.samplingRateHz = null;        // optional, for extrapolation
  }

  resetTimestamps() {
    this.lastTicksUnwrapped = 0;
    this.cycle = 0;
    this.systemOffsetFirstTime = null;
  }

  unwrapTicks(ticks) {
    let unwrapped = ticks + (SensorBase.TICKS_MAX_VALUE * this.cycle);
    if (this.lastTicksUnwrapped > unwrapped) {
      this.cycle += 1;
      unwrapped = ticks + (SensorBase.TICKS_MAX_VALUE * this.cycle);
    }
    this.lastTicksUnwrapped = unwrapped;
    return unwrapped;
  }

  ticksToMillis(unwrappedTicks) {
    // ticks/sec = 32768 => ticks per ms = 32.768
    return (unwrappedTicks / SensorBase.CLOCK_FREQ) * 1000.0;
  }

  getTimestampUnwrappedMillis(lastSampleTicksU24, systemMillis) {
    const unwrappedTicks = this.unwrapTicks(lastSampleTicksU24);
    const shimmerMillis = this.ticksToMillis(unwrappedTicks);

    // store first-time system offset
    if (this.systemOffsetFirstTime == null) {
      this.systemOffsetFirstTime = systemMillis - shimmerMillis;
    }
    return { shimmerMillis, systemOffsetFirstTime: this.systemOffsetFirstTime };
  }

  extrapolateSampleTimes({
    numSamples,
    i, // 0..numSamples-1
    samplingRateHz,
    tsLastSampleMillis,
    systemTsLastSampleMillis,
    systemOffsetFirstTime
  }) {
    const sr = samplingRateHz ?? this.samplingRateHz;
    if (!sr || sr <= 0) {
      return {
        tsMillis: tsLastSampleMillis,
        systemTsMillis: systemTsLastSampleMillis,
        systemTsPlotMillis: (systemOffsetFirstTime != null)
          ? tsLastSampleMillis + systemOffsetFirstTime
          : systemTsLastSampleMillis
      };
    }
    const sampleOffsetSec = (numSamples - i - 1) / sr;
    const tsMillis = tsLastSampleMillis - (sampleOffsetSec * 1000);
    const systemTsMillis = systemTsLastSampleMillis - (sampleOffsetSec * 1000);
    const systemTsPlotMillis = (systemOffsetFirstTime != null)
      ? tsMillis + systemOffsetFirstTime
      : systemTsMillis;
    return { tsMillis, systemTsMillis, systemTsPlotMillis };
  }
}

// ---------- sensor: LIS2DW12 accel (id=2) ----------
class SensorLIS2DW12 extends SensorBase {
  constructor() {
    super();
    // from SensorLIS2DW12.cs
    this.offset = [0, 0, 0];
    this.align = [
      [0, 0, 1],
      [1, 0, 0],
      [0, 1, 0]
    ];
    this.sensitivityByRange = {
      "2G":  [1671.665922915, 1671.665922915, 1671.665922915],
      "4G":  [835.832961457,  835.832961457,  835.832961457],
      "8G":  [417.916480729,  417.916480729,  417.916480729],
      "16G": [208.958240364,  208.958240364,  208.958240364]
    };
    this.range = "2G";           // default
    this.samplingRateHz = 50;    // set this to your actual config
  }

  setRange(rangeStr /* "2G"|"4G"|"8G"|"16G" */) {
    if (this.sensitivityByRange[rangeStr]) this.range = rangeStr;
  }

  _calibrate(raw) {
    // CalibrateInertialSensorData( raw, ALIGN, SENS, OFFSET )
    const v = [
      raw[0] - this.offset[0],
      raw[1] - this.offset[1],
      raw[2] - this.offset[2]
    ];
    // alignment matrix multiply: out = A * v
    const a = this.align;
    const aligned = [
      a[0][0]*v[0] + a[0][1]*v[1] + a[0][2]*v[2],
      a[1][0]*v[0] + a[1][1]*v[1] + a[1][2]*v[2],
      a[2][0]*v[0] + a[2][1]*v[1] + a[2][2]*v[2],
    ];
    const s = this.sensitivityByRange[this.range];
    return [ aligned[0] / s[0], aligned[1] / s[1], aligned[2] / s[2] ]; // m/s^2
  }

  parsePayload(sensorPayloadBytes) {
    // sample = 6 bytes: x,y,z int16 LE per sample
    const BYTES_PER_SAMPLE = 6;
    const n = Math.floor(sensorPayloadBytes.length / BYTES_PER_SAMPLE);
    const out = [];
    for (let i = 0; i < n; i++) {
      const off = i * BYTES_PER_SAMPLE;
      const raw = [
        i16le(sensorPayloadBytes, off + 0),
        i16le(sensorPayloadBytes, off + 2),
        i16le(sensorPayloadBytes, off + 4),
      ];
      const cal = this._calibrate(raw);
      out.push({ raw, cal, units: { cal: "m/s^2" } });
    }
    return out;
  }
}

// ---------- sensor: LSM6DS3 accel/gyro (id=3) ----------
class SensorLSM6DS3 extends SensorBase {
  constructor() {
    super();
    // from SensorLSM6DS3.cs
    this.offset = [0, 0, 0];
    this.align = [
      [0, 0, 1],
      [-1, 0, 0],
      [0, -1, 0]
    ];

    // LSB per unit (diagonal), so unit = raw / diag
    this.accSensByRange = {
      "2G":  [1671.665922915, 1671.665922915, 1671.665922915],
      "4G":  [835.832961457,  835.832961457,  835.832961457],
      "8G":  [417.916480729,  417.916480729,  417.916480729],
      "16G": [208.958240364,  208.958240364,  208.958240364]
    };
    this.gyroSensByRange = {
      "250DPS":  [114.285714286, 114.285714286, 114.285714286],
      "500DPS":  [57.142857143,  57.142857143,  57.142857143],
      "1000DPS": [28.571428571,  28.571428571,  28.571428571],
      "2000DPS": [14.285714286,  14.285714286,  14.285714286]
    };

    this.accRange = "2G";
    this.gyroRange = "250DPS";
    this.accEnabled = true;
    this.gyroEnabled = true;
    this.samplingRateHz = 50; // set to your actual config
  }

  setAccelEnabled(v) { this.accEnabled = !!v; }
  setGyroEnabled(v) { this.gyroEnabled = !!v; }
  setAccelRange(r) { if (this.accSensByRange[r]) this.accRange = r; }
  setGyroRange(r) { if (this.gyroSensByRange[r]) this.gyroRange = r; }

  _applyAlignAndOffset(raw3) {
    const v = [
      raw3[0] - this.offset[0],
      raw3[1] - this.offset[1],
      raw3[2] - this.offset[2]
    ];
    const a = this.align;
    return [
      a[0][0]*v[0] + a[0][1]*v[1] + a[0][2]*v[2],
      a[1][0]*v[0] + a[1][1]*v[1] + a[1][2]*v[2],
      a[2][0]*v[0] + a[2][1]*v[1] + a[2][2]*v[2],
    ];
  }

  parsePayload(sensorPayloadBytes) {
    // C# layout:
    // - If gyro && accel enabled: accel is at +6, gyro at +0 (so bytes/sample=12)
    // - Else: accel alone => 6, gyro alone => 6
    let bytesPerSample = 6;
    if (this.gyroEnabled && this.accEnabled) bytesPerSample = 12;

    const n = Math.floor(sensorPayloadBytes.length / bytesPerSample);
    const out = [];

    for (let i = 0; i < n; i++) {
      const base = i * bytesPerSample;

      let gyroRaw = null, gyroCal = null;
      let accRaw = null, accCal = null;

      if (this.gyroEnabled && this.accEnabled) {
        gyroRaw = [ i16le(sensorPayloadBytes, base + 0), i16le(sensorPayloadBytes, base + 2), i16le(sensorPayloadBytes, base + 4) ];
        accRaw  = [ i16le(sensorPayloadBytes, base + 6), i16le(sensorPayloadBytes, base + 8), i16le(sensorPayloadBytes, base + 10) ];
      } else if (this.gyroEnabled) {
        gyroRaw = [ i16le(sensorPayloadBytes, base + 0), i16le(sensorPayloadBytes, base + 2), i16le(sensorPayloadBytes, base + 4) ];
      } else if (this.accEnabled) {
        accRaw  = [ i16le(sensorPayloadBytes, base + 0), i16le(sensorPayloadBytes, base + 2), i16le(sensorPayloadBytes, base + 4) ];
      }

      if (accRaw) {
        const aligned = this._applyAlignAndOffset(accRaw);
        const s = this.accSensByRange[this.accRange];
        accCal = [ aligned[0]/s[0], aligned[1]/s[1], aligned[2]/s[2] ];
      }
      if (gyroRaw) {
        const aligned = this._applyAlignAndOffset(gyroRaw);
        const s = this.gyroSensByRange[this.gyroRange];
        gyroCal = [ aligned[0]/s[0], aligned[1]/s[1], aligned[2]/s[2] ];
      }

      out.push({
        accel: accRaw ? { raw: accRaw, cal: accCal, units: "m/s^2" } : null,
        gyro:  gyroRaw ? { raw: gyroRaw, cal: gyroCal, units: "deg/s" } : null
      });
    }
    return out;
  }
}

// ---------- sensor: PPG (id=4) ----------
class SensorPPG extends SensorBase {
  constructor() {
    super();
    // enabled channels
    this.red = false;
    this.ir = false;
    this.green = false;
    this.blue = false;

    // from SensorPPG.cs
    this.adcLsb = [7.8125, 15.625, 31.25, 62.5];
    this.adcBitShift = [2 ** 7, 2 ** 6, 2 ** 5, 2 ** 4];
    this.adcResolutionIndex = 0; // 0..3

    this.samplingRateHz = 50; // set to your actual config
  }

  setChannels({ red, ir, green, blue }) {
    if (typeof red === "boolean") this.red = red;
    if (typeof ir === "boolean") this.ir = ir;
    if (typeof green === "boolean") this.green = green;
    if (typeof blue === "boolean") this.blue = blue;
  }

  setAdcResolutionIndex(i /*0..3*/) {
    if (i >= 0 && i <= 3) this.adcResolutionIndex = i;
  }

  calibrateValue(uncalValue) {
    // C#:
    // cal = (uncal / bitShift[idx]) * lsb[idx] / 1000
    const idx = this.adcResolutionIndex;
    return (uncalValue / this.adcBitShift[idx]) * this.adcLsb[idx] / 1000.0;
  }

  parsePayload(sensorPayloadBytes) {
    // Element order in C#:
    // 1 PPG_RED, 2 PPG_IR, 3 PPG_GREEN, 4 PPG_BLUE
    const enabled = [];
    if (this.red) enabled.push("RED");
    if (this.ir) enabled.push("IR");
    if (this.green) enabled.push("GREEN");
    if (this.blue) enabled.push("BLUE");

    const bytesPerSample = enabled.length * 3;
    if (bytesPerSample === 0) return [];

    const n = Math.floor(sensorPayloadBytes.length / bytesPerSample);
    const out = [];

    for (let i = 0; i < n; i++) {
      const base = i * bytesPerSample;
      let off = 0;
      const sample = {};
      for (const ch of enabled) {
        const b0 = sensorPayloadBytes[base + off + 0];
        const b1 = sensorPayloadBytes[base + off + 1];
        const b2 = sensorPayloadBytes[base + off + 2];
        off += 3;

        // 3-byte int, then mask 0x7FFFF like C#
        let uncal = (b0 | (b1 << 8) | (b2 << 16)) >>> 0;
        uncal &= 0x7FFFF;

        sample[ch] = {
          raw: uncal,
          cal: this.calibrateValue(uncal),
          // C# stores "NanoAmpere" for CAL. Keep the numeric value + let you label it in UI.
          units: { raw: "counts", cal: "scaled" }
        };
      }
      out.push(sample);
    }
    return out;
  }
}

// ---------- sensor: GSR (id=1) ----------
class SensorGSR extends SensorBase {
  constructor() {
    super();
    // from SensorGSR.cs
    this.LIMIT_MIN_VALID_USIEMENS = 0.03;
    this.GSR_UNCAL_LIMIT_RANGE3_SR68 = 1134;
    this.GSR_UNCAL_LIMIT_RANGE3_SR62 = 683;

    this.SHIMMER3_REF_KOHMS = [40.2, 287.0, 1000.0, 3300.0];
    this.SR68_REF_KOHMS     = [21.0, 150.0, 562.0, 1740.0];

    // user-config
    this.gsrEnabled = true;
    this.battEnabled = false;
    this.gsrRangeSetting = 4; // 0..3 fixed, 4 auto (matches C#)
    this.hardwareIdentifier = "VERISENSE_PULSE_PLUS"; // or "VERISENSE_GSR_PLUS" or other

    this.samplingRateHz = 50; // set to your actual config
  }

  setHardwareIdentifier(idStr) { this.hardwareIdentifier = idStr; }
  setEnabled({ gsr, batt }) {
    if (typeof gsr === "boolean") this.gsrEnabled = gsr;
    if (typeof batt === "boolean") this.battEnabled = batt;
  }
  setGsrRangeSetting(v /*0..4*/) { this.gsrRangeSetting = v; }

  calibrateAdcToVolts(uncal12bit) {
    // from VerisenseDevice.CalibrateADCValueToVolts
    const adcRange = (2 ** 12) - 1;
    let refVoltage = (1.8 / 4.0);
    if (this.hardwareIdentifier === "VERISENSE_GSR_PLUS") {
      refVoltage = (3.0 / 4.0);
    }
    const adcScaling = (1.0 / 4.0);
    const adcOffset = 0;
    return ((((uncal12bit - adcOffset) * refVoltage) / adcRange) / adcScaling);
  }

  calibrateGsrToKOhmsUsingAmplifierEq(volts, range /*0..3*/) {
    // from SensorGSR.CalibrateGsrDataToKOhmsUsingAmplifierEq
    let rFeedback = this.SHIMMER3_REF_KOHMS[range];
    if (this.hardwareIdentifier === "VERISENSE_PULSE_PLUS") {
      rFeedback = this.SR68_REF_KOHMS[range];
    }
    let gsr_ref_voltage = 0.5;
    if (this.hardwareIdentifier === "VERISENSE_PULSE_PLUS") {
      gsr_ref_voltage = 0.4986;
    }
    const rSource = rFeedback / ((volts / gsr_ref_voltage) - 1.0);
    return rSource;
  }

  nudgeGsrResistance(kOhms) {
    // approximate to the configured range limits (like UtilCalibration.NudgeDouble)
    const limitsByRange = {
      0: [8.0, 63.0],
      1: [63.0, 220.0],
      2: [220.0, 680.0],
      3: [680.0, 4700.0],
      4: [8.0, 4700.0]
    };
    const lim = limitsByRange[this.gsrRangeSetting] ?? [8.0, 4700.0];
    return Math.min(Math.max(kOhms, lim[0]), lim[1]);
  }

  kOhmToUSiemens(kOhms) { return 1000.0 / kOhms; }

  parsePayload(sensorPayloadBytes) {
    // sample is 2 bytes (GSR only) or 4 bytes (Batt + GSR), both int16 but only 12-bit ADC used
    const bytesPerSample = (this.gsrEnabled && this.battEnabled) ? 4 : 2;
    const n = Math.floor(sensorPayloadBytes.length / bytesPerSample);
    const out = [];

    for (let i = 0; i < n; i++) {
      const base = i * bytesPerSample;

      let batt = null;
      let gsr = null;

      let gsrStart = 0;
      let battStart = 0;
      if (this.battEnabled && this.gsrEnabled) gsrStart = 2;

      if (this.gsrEnabled) {
        const gsrraw = i16le(sensorPayloadBytes, base + gsrStart);
        let adc12 = (gsrraw & 0x0FFF);

        let currentRange = this.gsrRangeSetting;
        if (currentRange === 4) {
          currentRange = (gsrraw >> 14) & 0x03; // auto-range bits
        }

        // range-3 clamp differs by hardware
        if (currentRange === 3) {
          if (this.hardwareIdentifier === "VERISENSE_PULSE_PLUS") {
            if (adc12 < this.GSR_UNCAL_LIMIT_RANGE3_SR68) adc12 = this.GSR_UNCAL_LIMIT_RANGE3_SR68;
          } else if (this.hardwareIdentifier === "VERISENSE_GSR_PLUS") {
            if (adc12 < this.GSR_UNCAL_LIMIT_RANGE3_SR62) adc12 = this.GSR_UNCAL_LIMIT_RANGE3_SR62;
          }
        }

        const volts = this.calibrateAdcToVolts(adc12);
        let kOhms = this.calibrateGsrToKOhmsUsingAmplifierEq(volts, currentRange);
        kOhms = this.nudgeGsrResistance(kOhms);
        const uS = this.kOhmToUSiemens(kOhms);
        const connectivity = (uS > this.LIMIT_MIN_VALID_USIEMENS) ? "Connected" : "Disconnected";

        gsr = { raw: gsrraw, adc12, range: currentRange, volts, kOhms, uS, connectivity };
      }

      if (this.battEnabled) {
        const braw = i16le(sensorPayloadBytes, base + battStart) & 0x0FFF;
        // C# reports mV using CalibrateADCValueToMilliVolts
        const mv = this.calibrateAdcToVolts(braw) * 1000.0;
        batt = { raw: braw, mV: mv };
      }

      out.push({ gsr, batt });
    }

    return out;
  }
}

// ---------- Verisense device (Web Bluetooth) ----------
export class VerisenseBleDevice extends TinyEmitter {
  // NUS UUIDs (from your C#)
  static NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
  static NUS_TX      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"; // write
  static NUS_RX      = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"; // notify

  constructor({
    hardwareIdentifier = "VERISENSE_PULSE_PLUS",
    stripStreamCrc = true,
    verifyStreamCrc = false
  } = {}) {
    super();
    this.hardwareIdentifier = hardwareIdentifier;

    this.device = null;
    this.server = null;
    this.service = null;
    this.tx = null;
    this.rx = null;

    this._mode = "idle"; // "idle" | "streaming" | "command"
    this._newPayload = true;
    this._expectedLen = 0;
    this._buf = new Uint8Array(0);

    this.stripStreamCrc = !!stripStreamCrc;
    this.verifyStreamCrc = !!verifyStreamCrc;

    // sensor map (matches VerisenseDevice.CreateSensorMap)
    this.sensors = {
      1: new SensorGSR(),
      2: new SensorLIS2DW12(),
      3: new SensorLSM6DS3(),
      4: new SensorPPG()
    };
    this.sensors[1].setHardwareIdentifier(this.hardwareIdentifier);

    // pending command response
    this._pending = null; // { resolve, reject }
  }

  // quick access
  get gsr() { return this.sensors[1]; }
  get accel1() { return this.sensors[2]; }
  get gyroAccel2() { return this.sensors[3]; }
  get ppg() { return this.sensors[4]; }

  async connect({ filters, optionalServices } = {}) {
    const opts = {
      // Prefer namePrefix if you have it, else keep service filter.
      filters: filters ?? [{ services: [VerisenseBleDevice.NUS_SERVICE] }],
      optionalServices: optionalServices ?? [VerisenseBleDevice.NUS_SERVICE]
    };

    this.device = await navigator.bluetooth.requestDevice(opts);
    this.device.addEventListener("gattserverdisconnected", () => {
      this.emit("disconnected", {});
      this._mode = "idle";
    });

    this.server = await this.device.gatt.connect();
    this.service = await this.server.getPrimaryService(VerisenseBleDevice.NUS_SERVICE);
    this.tx = await this.service.getCharacteristic(VerisenseBleDevice.NUS_TX);
    this.rx = await this.service.getCharacteristic(VerisenseBleDevice.NUS_RX);

    await this.rx.startNotifications();
    this.rx.addEventListener("characteristicvaluechanged", (ev) => {
      const v = ev.target.value;
      const bytes = new Uint8Array(v.buffer.slice(v.byteOffset, v.byteOffset + v.byteLength));
      this._onRx(bytes);
    });

    this.emit("connected", { name: this.device.name, id: this.device.id });
    return true;
  }

  async disconnect() {
    try { if (this.rx) this.rx.stopNotifications?.(); } catch {}
    try { if (this.device?.gatt?.connected) this.device.gatt.disconnect(); } catch {}
    this._mode = "idle";
    this.emit("disconnected", {});
  }

  // ---------- requests ----------
  async writeBytes(bytes) {
    const u8 = (bytes instanceof Uint8Array) ? bytes : new Uint8Array(bytes);
    if (!this.tx) throw new Error("Not connected");
    // Prefer withoutResponse if available (matches your C#)
    if (this.tx.writeValueWithoutResponse) {
      await this.tx.writeValueWithoutResponse(u8);
    } else {
      await this.tx.writeValue(u8);
    }
  }

  _makeReq(opcode, payloadBytes = []) {
    const p = (payloadBytes instanceof Uint8Array) ? payloadBytes : new Uint8Array(payloadBytes);
    const len = p.length;
    const out = new Uint8Array(3 + len);
    out[0] = opcode & 0xFF;
    out[1] = len & 0xFF;
    out[2] = (len >> 8) & 0xFF;
    out.set(p, 3);
    return out;
  }

  // Generic: send request and wait for next complete command payload
  async request(opcode, payloadBytes = [], timeoutMs = 3000) {
    if (this._pending) throw new Error("A request is already pending");
    this._mode = "command";
    this._resetAssembler();

    const req = this._makeReq(opcode, payloadBytes);

    const p = new Promise((resolve, reject) => {
      const t = setTimeout(() => {
        this._pending = null;
        reject(new Error("Request timeout"));
      }, timeoutMs);

      this._pending = {
        resolve: (x) => { clearTimeout(t); this._pending = null; resolve(x); },
        reject: (e) => { clearTimeout(t); this._pending = null; reject(e); }
      };
    });

    await this.writeBytes(req);
    return p;
  }

  // Convenience (from VerisenseBLEDevice.cs)
  readStatus()           { return this.request(0x11); }
  readStatus2()          { return this.request(0x1C); }
  readProductionConfig() { return this.request(0x13); }
  readOperationalConfig(){ return this.request(0x14); }
  readTime()             { return this.request(0x15); }
  readPendingEvents()    { return this.request(0x17); }
  disconnectRequest()    { return this.request(0x2B); }

  async startStreaming() {
    // StreamDataRequest = [0x2A, 0x01, 0x00, 0x01]
    this._mode = "streaming";
    this._resetAssembler();
    await this.writeBytes(this._makeReq(0x2A, [0x01]));
    this.emit("streaming", { on: true });
  }

  async stopStreaming() {
    // StopStreamRequest = [0x2A, 0x01, 0x00, 0x02]
    await this.writeBytes(this._makeReq(0x2A, [0x02]));
    this._mode = "idle";
    this.emit("streaming", { on: false });
  }

  // ---------- RX parsing / reassembly ----------
  _resetAssembler() {
    this._newPayload = true;
    this._expectedLen = 0;
    this._buf = new Uint8Array(0);
  }

  _appendBuf(chunk) {
    const merged = new Uint8Array(this._buf.length + chunk.length);
    merged.set(this._buf, 0);
    merged.set(chunk, this._buf.length);
    this._buf = merged;
  }

  _onRx(bytes) {
    // Streaming special case seen in C#: 0x4A 00 00 (length=0), ignore.
    if (this._mode === "streaming" && bytes.length === 3 && bytes[0] === 0x4A) return;

    // Assemble: first chunk has [hdr][lenLE16] then payload bytes; later chunks are raw continuation.
    if (this._newPayload) {
      if (bytes.length < 3) return; // ultra-rare; ignore
      const len = u16le(bytes[1], bytes[2]);
      this._expectedLen = len;
      this._buf = new Uint8Array(0);
      if (bytes.length > 3) this._appendBuf(bytes.slice(3));
      this._newPayload = false;
    } else {
      this._appendBuf(bytes);
    }

    if (this._buf.length >= this._expectedLen) {
      const payload = this._buf.slice(0, this._expectedLen);
      // reset for next packet
      this._resetAssembler();

      if (this._mode === "streaming") {
        this._handleStreamingPayload(payload);
      } else {
        // command payload complete
        if (this._pending) this._pending.resolve({ payload });
        this.emit("commandPayload", { payload });
      }
    }
  }

  _handleStreamingPayload(payload) {
    // payload starts with: [sensorId][tick u24][sensorPayload...][optional crc16]
    if (payload.length < 4) return;

    let body = payload;
    let crcOk = null;

    if (this.stripStreamCrc && payload.length >= 6) {
      // assume last 2 bytes are CRC16 (little-endian)
      const claimed = (payload[payload.length - 2] | (payload[payload.length - 1] << 8)) >>> 0;
      const dataNoCrc = payload.slice(0, payload.length - 2);

      if (this.verifyStreamCrc) {
        const calc = crc16_ccitt_false(dataNoCrc);
        crcOk = (calc === claimed);
      }

      body = dataNoCrc;

      if (this.verifyStreamCrc && crcOk === false) {
        this.emit("streamCrcFail", { claimed, body: dataNoCrc });
        // still continue emitting raw+decoded so you can decide what to do
      }
    }

    const sensorId = body[0];
    const tick = u24le(body, 1);
    const sensorPayload = body.slice(4);

    const sensor = this.sensors[sensorId];
    const systemTsLastSampleMillis = nowMillis();
    let tsInfo = null;
    if (sensor) tsInfo = sensor.getTimestampUnwrappedMillis(tick, systemTsLastSampleMillis);

    let decodedSamples = null;
    if (sensor) decodedSamples = sensor.parsePayload(sensorPayload);

    // optional timestamp extrapolation per sample
    let samplesWithTime = decodedSamples;
    if (sensor && Array.isArray(decodedSamples) && decodedSamples.length > 0 && tsInfo) {
      const num = decodedSamples.length;
      samplesWithTime = decodedSamples.map((s, i) => {
        const t = sensor.extrapolateSampleTimes({
          numSamples: num,
          i,
          samplingRateHz: sensor.samplingRateHz,
          tsLastSampleMillis: tsInfo.shimmerMillis,
          systemTsLastSampleMillis,
          systemOffsetFirstTime: tsInfo.systemOffsetFirstTime
        });
        return { ...s, timestamps: t };
      });
    }

    const packet = {
      sensorId,
      tick_u24: tick,
      decoded: samplesWithTime,
      rawPayload: sensorPayload,
      crcOk
    };

    this.emit("streamPacket", packet);
    this.emit("data", packet); // alias
  }
}

/* ------------------ example usage ------------------

import { VerisenseBleDevice } from "./verisense.js";

const v = new VerisenseBleDevice({
  hardwareIdentifier: "VERISENSE_PULSE_PLUS",
  stripStreamCrc: true,
  verifyStreamCrc: false
});

// configure decoders to match your op config
v.gyroAccel2.setAccelEnabled(true);
v.gyroAccel2.setGyroEnabled(true);
v.gyroAccel2.setAccelRange("2G");
v.gyroAccel2.setGyroRange("250DPS");

v.ppg.setChannels({ red: true, ir: true, green: false, blue: false });
v.ppg.setAdcResolutionIndex(0);

v.on("data", (pkt) => {
  // pkt.sensorId: 1/2/3/4
  // pkt.decoded: array of samples (each sample may include timestamps)
  console.log(pkt.sensorId, pkt.decoded?.[0]);
});

await v.connect();
await v.startStreaming();

----------------------------------------------------- */
