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

function normalizeOperationalConfig(payload) {
  if (!payload) return null;
  if (payload instanceof Uint8Array) return payload;
  if (payload instanceof ArrayBuffer) return new Uint8Array(payload);
  if (Array.isArray(payload)) return new Uint8Array(payload);
  // DataView / TypedArray
  if (payload.buffer) return new Uint8Array(payload.buffer, payload.byteOffset ?? 0, payload.byteLength ?? payload.buffer.byteLength);
  throw new Error("normalizeOperationalConfig: unsupported payload type");
}

const OP_IDX = Object.freeze({
  GEN_CFG_0: 1,
  GEN_CFG_1: 2,
  GEN_CFG_2: 3,
  GEN_CFG_3: 4,

  ACCEL1_CFG_0: 5,
  ACCEL1_CFG_1: 6,
  ACCEL1_CFG_2: 7,
  ACCEL1_CFG_3: 8,

  GYRO_ACCEL2_CFG_0: 10,
  GYRO_ACCEL2_CFG_1: 11,
  GYRO_ACCEL2_CFG_2: 12,
  GYRO_ACCEL2_CFG_3: 13,
  GYRO_ACCEL2_CFG_4: 14,
  GYRO_ACCEL2_CFG_5: 15,
  GYRO_ACCEL2_CFG_6: 16,
  GYRO_ACCEL2_CFG_7: 17,

  START_TIME: 21,
  END_TIME: 25,
  INACTIVE_TIMEOUT: 29,
  BLE_RETRY_COUNT: 30,
  BLE_TX_POWER: 31,
  BLE_DATA_TRANS_WKUP_INT_HRS: 32,
  BLE_DATA_TRANS_WKUP_TIME: 33,
  BLE_DATA_TRANS_WKUP_DUR: 35,
  BLE_DATA_TRANS_RETRY_INT: 36,
  BLE_STATUS_WKUP_INT_HRS: 38,
  BLE_STATUS_WKUP_TIME: 39,
  BLE_STATUS_WKUP_DUR: 41,
  BLE_STATUS_RETRY_INT: 42,
  BLE_RTC_SYNC_WKUP_INT_HRS: 44,
  BLE_RTC_SYNC_WKUP_TIME: 45,
  BLE_RTC_SYNC_WKUP_DUR: 47,
  BLE_RTC_SYNC_RETRY_INT: 48,

  ADC_CHANNEL_SETTINGS_0: 50,
  ADC_CHANNEL_SETTINGS_1: 51,
  ADAPTIVE_SCHEDULER_INT: 52,
  ADAPTIVE_SCHEDULER_FAILCOUNT_MAX: 54,
  PPG_REC_DUR_SECS_LSB: 55,
  PPG_REC_DUR_SECS_MSB: 56,
  PPG_REC_INT_MINS_LSB: 57,
  PPG_REC_INT_MINS_MSB: 58,
  PPG_FIFO_CONFIG: 59,
  PPG_MODE_CONFIG2: 60,
  PPG_MA_DEFAULT: 61,
  PPG_MA_MAX_RED_IR: 62,
  PPG_MA_MAX_GREEN_BLUE: 63,
  PPG_AGC_TARGET_PERCENT_OF_RANGE: 64,
  PPG_MA_LED_PILOT: 66,
  PPG_DAC1_CROSSTALK: 67,
  PPG_DAC2_CROSSTALK: 68,
  PPG_DAC3_CROSSTALK: 69,
  PPG_DAC4_CROSSTALK: 70,
  PROX_AGC_MODE: 71
});

const READ_DATA_REQ = new Uint8Array([0x12, 0x00, 0x00]);
const DATA_ACK      = new Uint8Array([0x82, 0x00, 0x00]);
const DATA_NACK     = new Uint8Array([0x72, 0x00, 0x00]);
const DATA_EOS_HDR  = 0x42;

function u16le_at(bytes, off) {
  return (bytes[off] | (bytes[off + 1] << 8)) >>> 0;
}

function getOriginalCrcLE(payload) {
  const n = payload.length;
  return (payload[n - 2] | (payload[n - 1] << 8)) >>> 0;
}

function computeCrcLikeCSharp(payload) {
  // C# ComputeCRC iterates to length-2 (excludes CRC bytes)
  const n = payload.length;
  const data = payload.subarray(0, n - 2);
  return crc16_ccitt_false(data);
}

// safe byte getter
function b(op, idx) {
  if (!op || idx == null) return null;
  if (idx < 0 || idx >= op.length) return null;
  return op[idx] & 0xFF;
}

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
  
    applyOperationalConfig(op) {
  console.log("[LIS2DW12] applyOperationalConfig()");
  console.log("[LIS2DW12] op len =", op?.length, "op[0]=", op ? `0x${op[0].toString(16)}` : "(null)");
  console.log("GEN_CFG_0", OP_IDX.GEN_CFG_0);
  console.log("ACCEL1_CFG_0", OP_IDX.ACCEL1_CFG_0);
  console.log("ACCEL1_CFG_1", OP_IDX.ACCEL1_CFG_1);
  console.log("ACCEL1_CFG_2", OP_IDX.ACCEL1_CFG_2);

  const gen0 = b(op, OP_IDX.GEN_CFG_0);
  const cfg0 = b(op, OP_IDX.ACCEL1_CFG_0);
  const cfg1 = b(op, OP_IDX.ACCEL1_CFG_1);

  if (gen0 == null || cfg0 == null || cfg1 == null) {
    console.warn("[LIS2DW12] Missing required bytes; cannot apply config.");
    return;
  }

  this.enabled = ((gen0 >> 7) & 0x01) === 1;

  const rangeSetting = (cfg1 >> 4) & 0x03;
  const modeSetting  = (cfg0 >> 2) & 0x03;
  const rateSetting  = (cfg0 >> 4) & 0x0F;

  console.log("[LIS2DW12] decoded bits:", {
    enabled: this.enabled,
    rangeSetting,
    modeSetting,
    rateSetting
  });

  const rangeMap = { 0: "2G", 1: "4G", 2: "8G", 3: "16G" };
  const nextRange = rangeMap[rangeSetting] ?? "2G";
  const prevRange = this.range;

  this.setRange(nextRange);

  const lowPowerHzByCfg = { 1: 1.6, 2: 12.5, 3: 25, 4: 50, 5: 100, 6: 200 };
  const highPerfHzByCfg = { 1: 12.5, 3: 25, 4: 50, 5: 100, 6: 200, 7: 400, 8: 800, 9: 1600 };

  const isLowPower = (modeSetting === 0);
  const hz = isLowPower ? lowPowerHzByCfg[rateSetting] : highPerfHzByCfg[rateSetting];
  const prevHz = this.samplingRateHz;

  if (hz) this.samplingRateHz = hz;

  console.log("[LIS2DW12] applied:", {
    prevRange, nextRange: this.range,
    prevHz, nextHz: this.samplingRateHz,
    isLowPower
  });
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
  
  applyOperationalConfig(op) {
    // Enabled flags (GEN_CFG_0 bit6 accel2, bit5 gyro) :contentReference[oaicite:10]{index=10}
    this.accEnabled  = (op[OP_IDX.GEN_CFG_0] & 0b01000000) !== 0;
    this.gyroEnabled = (op[OP_IDX.GEN_CFG_0] & 0b00100000) !== 0;

    // Sampling rate nibble(s) live in GYRO_ACCEL2_CFG_4 (accel hi nibble, gyro low nibble)
    const cfg4 = op[OP_IDX.GYRO_ACCEL2_CFG_4];
    const accelRateCfg = (cfg4 >> 4) & 0x0F;
    const gyroRateCfg  = (cfg4 >> 0) & 0x0F;

    // Config-value -> Hz mapping (match your SamplingRate.Settings)
    const hzByCfg = {
      0: null,
      1: 12.5,
      2: 26,
      3: 52,
      4: 104,
      5: 208,
      6: 416,
      7: 833,
      8: 1660
    };

    // Range bits live in GYRO_ACCEL2_CFG_5
    const cfg5 = op[OP_IDX.GYRO_ACCEL2_CFG_5];
    const accelRangeCfg = (cfg5 >> 2) & 0x03;      // :contentReference[oaicite:11]{index=11}
    const gyroRangeCfg  = (cfg5 >> 4) & 0x03;      // :contentReference[oaicite:12]{index=12}

    const accelRangeMap = { 0: "2G", 1: "4G", 2: "8G", 3: "16G" };
    const gyroRangeMap  = { 0: "250DPS", 1: "500DPS", 2: "1000DPS", 3: "2000DPS" };

    this.setAccelRange(accelRangeMap[accelRangeCfg] ?? this.accRange);
    this.setGyroRange(gyroRangeMap[gyroRangeCfg] ?? this.gyroRange);

    // In your C# there’s effectively “one ODR”, but accel/gyro fields exist; pick accel as main
    const hz = hzByCfg[accelRateCfg] ?? hzByCfg[gyroRateCfg];
    if (hz) this.samplingRateHz = hz;
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

// Operational Config → sensor settings (ported from C# SensorGSR.InitializeUsingOperationConfig)
  applyOperationalConfig(op) {
	console.log("gsr" + op);  
    if (!op) return;

    const gen1 = op[OP_IDX.GEN_CFG_1] ?? 0;
    const gen2 = op[OP_IDX.GEN_CFG_2] ?? 0;

    // C#: if ((GEN_CFG_1 >> 7) == 1) GSREnabled = true;
    this.gsrEnabled = ((gen1 >> 7) & 0x01) === 1;

    // C#: if ((GEN_CFG_2 & 0b00000010) > 1) BattEnabled = true;
    this.battEnabled = (gen2 & 0b00000010) !== 0;

    // Rate/range/oversampling are stored for debugging (mapping tables can be added later)
    const rateCfg = (op[OP_IDX.ADC_CHANNEL_SETTINGS_0] ?? 0) & 0x3F;
    const cfg1 = (op[OP_IDX.ADC_CHANNEL_SETTINGS_1] ?? 0) & 0xFF;
    const rangeCfg = cfg1>>5 & 0x07;
    const oversamplingCfg = (cfg1 >> 4) & 0x0F;

    this.gsrRateSettingRaw = rateCfg;
    this.gsrRangeSettingRaw = rangeCfg;
    this.gsrOversamplingRateSettingRaw = oversamplingCfg;

    // In C#, rangeCfg is mapped through GSRRange.Settings. In practice this is usually 0..4.
    // We keep it conservative: accept 0..4, otherwise leave current setting unchanged.
    if (rangeCfg >= 0 && rangeCfg <= 4) {
      this.gsrRangeSetting = rangeCfg;
    }

    // NOTE: samplingRateHz mapping is firmware/settings-table specific.
    // We leave this.samplingRateHz as-is, but you can log rateCfg to build a table.
  }

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
		console.log("gsrraw: " + gsrraw);
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
		console.log(gsr);
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
	this._sync = null;
    this._rxStreamBuf = new Uint8Array(0);

    // ---- transport ----
    // Default: BLE (Web Bluetooth). Optional: USB serial (Web Serial).
    this._transportKind = null; // 'ble' | 'serial'
    this.port = null;           // Web Serial port (if using serial)
    this._serialAbort = null;   // AbortController for read loop
    this._onGattDisconnected = null; // stored BLE disconnect handler

    this.device = null;
    this.server = null;
    this.service = null;
    this.tx = null;
    this.rx = null;
	this.debugSync = true;
	this._syncRxCount = 0;
	this._syncPayloadCount = 0;

    this._loggedChain = Promise.resolve();
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

  async connect({ device = null, filters, optionalServices } = {}) {
  console.log("CONNECT");


    // If we're currently connected over serial, tear that down first so UI can use one device instance.
    if (this._transportKind === "serial" || this.port) {
      try { await this.disconnect({ reason: "Switching transport" }); } catch {}
    }

    this._transportKind = "ble";
  const opts = {
    filters: filters ?? [{ services: [VerisenseBleDevice.NUS_SERVICE] }],
    optionalServices: optionalServices ?? [VerisenseBleDevice.NUS_SERVICE]
  };

  this.device = device ?? await navigator.bluetooth.requestDevice(opts);

    // Keep a removable handler so manual disconnect doesn't double-emit.
    try {
      if (this._onGattDisconnected && this.device) {
        this.device.removeEventListener("gattserverdisconnected", this._onGattDisconnected);
      }
    } catch {}

    this._onGattDisconnected = () => {
      this._mode = "idle";
      this._transportKind = null;
      this.emit("disconnected", { kind: "ble" });
    };

    this.device.addEventListener("gattserverdisconnected", this._onGattDisconnected);

    this.server = await this.device.gatt.connect();
    this.service = await this.server.getPrimaryService(VerisenseBleDevice.NUS_SERVICE);
    this.tx = await this.service.getCharacteristic(VerisenseBleDevice.NUS_TX);
    this.rx = await this.service.getCharacteristic(VerisenseBleDevice.NUS_RX);

    await this.rx.startNotifications();
    this.rx.addEventListener("characteristicvaluechanged", (ev) => {
      const v = ev.target.value;
      const bytes = new Uint8Array(v.buffer.slice(v.byteOffset, v.byteOffset + v.byteLength));
      this._feedStreamBytes(bytes)
    });
	console.log(`CONNECT`);
    this.emit("connected", { name: this.device.name, id: this.device.id });
	
	 // ---- NEW: read op config + apply it to sensors ----
try {
  console.log("[opcfg] requesting readOperationalConfig (0x14)...");
  const t0 = performance.now();

  const rsp = await this.readOperationalConfig(); // { payload }
  const dt = (performance.now() - t0).toFixed(1);

  const payload = rsp?.payload;
  console.log(`[opcfg] response received in ${dt} ms`);
  console.log("[opcfg] payload type:", payload?.constructor?.name);
  console.log("[opcfg] payload len:", payload?.length);

  // hex dump helper (inline)
  const toHex = (u8, max = 96) => {
    if (!u8) return "(null)";
    const a = Array.from(u8.slice(0, max)).map(b => b.toString(16).padStart(2, "0")).join(" ");
    return u8.length > max ? `${a} … (+${u8.length - max} bytes)` : a;
  };

  console.log("[opcfg] payload head:", toHex(payload, 96));

  const op = normalizeOperationalConfig(payload);
  console.log("[opcfg] normalized len:", op?.length);
  console.log("[opcfg] normalized head:", toHex(op, 96));
  console.log("[opcfg] normalized startsWith 0x5A:", op?.[0] === 0x5A);

  // If you have OP_IDX defined, log the important bytes
  if (typeof OP_IDX !== "undefined") {
    const safe = (idx) => (op && idx >= 0 && idx < op.length) ? op[idx] : null;
    const logByte = (name, idx) => {
      const v = safe(idx);
      console.log(`[opcfg] ${name} @${idx}:`, v == null ? "OUT_OF_RANGE" : `0x${v.toString(16).padStart(2,"0")} (${v})`);
    };

    logByte("GEN_CFG_0", OP_IDX.GEN_CFG_0);
    logByte("ACCEL1_CFG_0", OP_IDX.ACCEL1_CFG_0);
    logByte("ACCEL1_CFG_1", OP_IDX.ACCEL1_CFG_1);
    logByte("ACCEL1_CFG_2", OP_IDX.ACCEL1_CFG_2);
    logByte("GYRO_ACCEL2_CFG_4", OP_IDX.GYRO_ACCEL2_CFG_4);
    logByte("GYRO_ACCEL2_CFG_5", OP_IDX.GYRO_ACCEL2_CFG_5);
  } else {
    console.warn("[opcfg] OP_IDX not defined; skipping per-field byte logging.");
  }

  this.operationalConfig = op;

  // Before/after snapshots
  console.log("[opcfg] accel1 BEFORE:", { range: this.accel1.range, hz: this.accel1.samplingRateHz, enabled: this.accel1.enabled });
  this.accel1.applyOperationalConfig(op);
  console.log("[opcfg] gsr BEFORE:" + this.gsr.gsrRangeSetting);
  this.gsr.applyOperationalConfig(op);
  console.log("[opcfg] gsr AFTER:" + this.gsr.gsrRangeSetting);
  console.log("[opcfg] accel1 AFTER :", { range: this.accel1.range, hz: this.accel1.samplingRateHz, enabled: this.accel1.enabled });

  console.log("[opcfg] gyroAccel2 BEFORE:", {
    accEnabled: this.gyroAccel2.accEnabled,
    gyroEnabled: this.gyroAccel2.gyroEnabled,
    accRange: this.gyroAccel2.accRange,
    gyroRange: this.gyroAccel2.gyroRange,
    hz: this.gyroAccel2.samplingRateHz
  });
  if (typeof this.gyroAccel2.applyOperationalConfig === "function") {
    this.gyroAccel2.applyOperationalConfig(op);
  } else {
    console.warn("[opcfg] gyroAccel2.applyOperationalConfig() is missing (did you paste it into the wrong class?)");
  }
  console.log("[opcfg] gyroAccel2 AFTER :", {
    accEnabled: this.gyroAccel2.accEnabled,
    gyroEnabled: this.gyroAccel2.gyroEnabled,
    accRange: this.gyroAccel2.accRange,
    gyroRange: this.gyroAccel2.gyroRange,
    hz: this.gyroAccel2.samplingRateHz
  });

  this.emit("opConfig", { op });
  console.log("[opcfg] done.");
} catch (e) {
  console.warn("[opcfg] FAILED:", e);
  this.emit("opConfigError", { error: String(e?.message ?? e), stack: e?.stack });
}

	
    return true;
  }

  // --- Web Serial (USB COM port) connect ---
  // Usage:
  //   const v = new VerisenseBleDevice(...);  // same protocol/core class
  //   await v.connectSerial({ baudRate: 115200 });
  //
  // Note: Web Serial is Chromium-based browsers only (Chrome/Edge).
  async connectSerial({
    port = null,
    baudRate = 115200,
    dataBits = 8,
    stopBits = 1,
    parity = "none",
    flowControl = "none",
    filters = null // e.g. [{ usbVendorId: 0x1234, usbProductId: 0x5678 }]
  } = {}) {
    if (!("serial" in navigator)) {
      throw new Error("Web Serial not supported in this browser. Use Chrome/Edge on HTTPS or http://localhost.");
    }

    // If already connected (BLE or Serial), disconnect first.
    if (this._transportKind === "ble" && this.device?.gatt?.connected) {
      await this.disconnect({ reason: "Reconnecting (serial)" });
    } else if (this._transportKind === "serial" && this.port) {
      await this.disconnect({ reason: "Reconnecting (serial)" });
    }

    this._transportKind = "serial";
    this._mode = "idle";
    this._resetAssembler();

    // Port selection (must be called from a user gesture)
    if (!port) {
      const opts = filters ? { filters } : undefined;
      port = await navigator.serial.requestPort(opts);
    }
    this.port = port;

    await this.port.open({ baudRate, dataBits, stopBits, parity, flowControl });

    // Kick off async read loop -> feeds _onRx()
    this._serialAbort = new AbortController();
    this._startSerialReadLoop(this._serialAbort.signal);

    this.emit("connected", { kind: "serial" });

    // Optionally: read operational config on connect (same as BLE)
    try {
      const rsp = await this.readOperationalConfig();
      const op = normalizeOperationalConfig(rsp?.payload);
	  
	  const toHex = (u8, max = 96) => {
	  if (!u8) return "(null)";
        const a = Array.from(u8.slice(0, max)).map(b => b.toString(16).padStart(2,"0")).join(" ");
        return u8.length > max ? `${a} … (+${u8.length - max} bytes)` : a;
      };

	  console.log("[opcfg][serial] payload type:", rsp?.payload?.constructor?.name);
      console.log("[opcfg][serial] payload len :", rsp?.payload?.length); 
      console.log("[opcfg][serial] payload head:", toHex(rsp?.payload, 96));
      console.log("[opcfg][serial] normalized len :", op?.length);
      console.log("[opcfg][serial] normalized head:", toHex(op, 96));

      if (op) {
        this.operationalConfig = op;
        this.accel1?.applyOperationalConfig?.(op);
        this.gyroAccel2?.applyOperationalConfig?.(op);
		this.gsr?.applyOperationalConfig?.(op);
        this.emit("opConfig", { op });
      }
    } catch (e) {
      this.emit("opConfigError", { error: String(e?.message ?? e), stack: e?.stack });
    }

    return true;
  }

  async _serialWrite(u8) {
    if (!this.port?.writable) throw new Error("Not connected");
    const writer = this.port.writable.getWriter();
    try {
      await writer.write(u8);
    } finally {
      writer.releaseLock();
    }
  }

  _startSerialReadLoop(signal) {
    const port = this.port;
    (async () => {
      try {
        while (port?.readable && !signal.aborted) {
          const reader = port.readable.getReader();
          try {
            while (!signal.aborted) {
              const { value, done } = await reader.read();
              if (done) break;
              if (value && value.length) this._feedStreamBytes(new Uint8Array(value));
            }
          } finally {
            reader.releaseLock();
          }
        }
      } catch (e) {
        if (!signal.aborted) console.warn("[serial] read loop error:", e);
      } finally {
        if (!signal.aborted) {
          this._mode = "idle";
          this.emit("disconnected", { kind: "serial" });
        }
      }
    })();
  }

  async _serialDisconnect() {
    try { this._serialAbort?.abort(); } catch {}
    try { await this.port?.close?.(); } catch {}
    this.port = null;
    this._serialAbort = null;
  }

// Unified disconnect for BLE + Web Serial.
// UI should always call: await v.disconnect()
async disconnect({ reason = null } = {}) {
  const kind = this._transportKind === "serial" ? "serial" : "ble";

  // Best-effort: stop streaming so device returns to idle
  if (this._mode === "streaming") {
    try { await this.stopStreaming(); } catch {}
  }

  // Abort any in-flight logged transfer
  if (this._sync) {
    try { this._abortSync(new Error(reason || "Disconnected")); } catch {}
  }

  // Tear down transport
  if (this._transportKind === "serial") {
    try { await this._serialDisconnect(); } catch {}
  } else {
    try { if (this.rx) await this.rx.stopNotifications?.(); } catch {}
    try { if (this._onGattDisconnected && this.device) this.device.removeEventListener("gattserverdisconnected", this._onGattDisconnected); } catch {}
    try { if (this.device?.gatt?.connected) this.device.gatt.disconnect(); } catch {}
  }

  // Clear handles (safe for both)
  this._mode = "idle";
  this._transportKind = null;

  this.port = null;
  this._serialAbort = null;

  this.tx = null;
  this.rx = null;
  this.service = null;
  this.server = null;
  this.device = null;

  this.emit("disconnected", { kind });
  return true;
}

async transferLoggedData({
  fileHandle = null,          // FileSystemFileHandle (Chromium) if you want true streaming-to-disk
  timeoutMs = 1000,
  maxNack = 5,
  maxCrcNack = 5,
  onProgress = null,          // ({ payloadIndex, bytesWritten, crcOk }) => void
} = {}) {
  const bleOk = !!(this.rx && this.tx);
  const serOk = !!(this.port?.readable && this.port?.writable);
  if (!bleOk && !serOk) throw new Error("Not connected");
  if (this._mode === "streaming") throw new Error("Stop streaming before TransferLoggedData");
  if (this._mode === "logged") throw new Error("Already syncing logged data");

  // ---- writer setup ----
  let writable = null;
  const chunks = []; // fallback if no fileHandle
  let bytesWritten = 0;

  if (fileHandle) {
    // Chromium/Edge: real incremental write to disk
    writable = await fileHandle.createWritable();
  }

  // ---- init sync session ----
  this._mode = "logged";
  this._resetAssembler();
  this._loggedChain = Promise.resolve();

  if (this.debugSync) {
	console.log("[sync] START transferLoggedData", {
	hasFileHandle: !!fileHandle,
	timeoutMs, maxNack, maxCrcNack,
	mode: this._mode
	});
  }
  this._syncRxCount = 0;
  this._syncPayloadCount = 0;


  const sync = {
    receiving: true,
    newPayload: true,
    lastReply: "NONE",
    nackCount: 0,
    nackCrcCount: 0,
    lastRxAt: Date.now(),
    timeoutMs,
    bytesWritten: 0,
    resolve: null,
    reject: null,
    timer: null,
    writable,
    chunks,
    onProgress,
  };
  this._sync = sync;

  const donePromise = new Promise((resolve, reject) => { sync.resolve = resolve; sync.reject = reject; });

  // ---- timeout loop (C# ProcessDataTimeout equivalent) ----
  sync.timer = setInterval(async () => {
    if (!this._sync?.receiving) return;
    const age = Date.now() - this._sync.lastRxAt;
    if (age < this._sync.timeoutMs) return;

    try {
      if (this._sync.lastReply === "NONE") {
        // resend read request
        await this.writeBytes(READ_DATA_REQ, { withResponse: true });
      } else {
        // force resend of last by NACK
		this._clearSyncRxBuffers("timeout-nack");
		await this.writeBytes(DATA_NACK);
        this._sync.nackCount++;
        this._sync.lastReply = "NACK";
        if (this._sync.nackCount >= maxNack) throw new Error("Too many NACK timeouts");
      }
      this._sync.lastRxAt = Date.now();
    } catch (e) {
      this._abortSync(e);
    }
  }, Math.max(250, Math.floor(timeoutMs / 2)));

  // ---- kick it off ----
  await this.writeBytes(READ_DATA_REQ, { withResponse: true });

  // ---- wait ----
  const result = await donePromise;

  // Make sure any queued payload writes have finished before closing/flushing
  await (this._loggedChain ?? Promise.resolve());

  // close writer if used
  if (writable) {
    await writable.close();
  }

  // if fallback: create blob
  if (!fileHandle) {
    const blob = new Blob(chunks, { type: "application/octet-stream" });
    return { ...result, blob };
  }

  return result;
}


  // ---------- requests ----------
  async writeBytes(bytes, { withResponse = false } = {}) {
    const u8 = (bytes instanceof Uint8Array) ? bytes : new Uint8Array(bytes);

    // Serial transport
    if (this._transportKind === "serial") {
      await this._serialWrite(u8);
      return;
    }

    // BLE transport (default)
    if (!this.tx) throw new Error("Not connected");

    // For sync/control frames (ACK/NACK/REQ), use write-with-response to get radio-level flow control.
    if (withResponse) {
      await this.tx.writeValue(u8);
      return;
    }

    // Bulk/normal writes can use withoutResponse if available (lower latency).
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
  _abortSync(err) {
  const s = this._sync;
  if (!s) return;
  s.receiving = false;
  if (s.timer) clearInterval(s.timer);
  this._sync = null;
  this._mode = "idle";
  s.reject(err instanceof Error ? err : new Error(String(err)));
}

_finishSync() {
  const s = this._sync;
  if (!s) return;
  s.receiving = false;
  if (s.timer) clearInterval(s.timer);
  const bytesWritten = s.bytesWritten;
  this._sync = null;
  this._mode = "idle";
  s.resolve({ ok: true, bytesWritten });
}

async _handleLoggedPayload(payloadU8) {
  this._syncPayloadCount++;

  if (this.debugSync) {
    const head = Array.from(payloadU8.slice(0, 10)).map(b => b.toString(16).padStart(2,"0")).join(" ");
    console.log(`[sync][payload#${this._syncPayloadCount}] len=${payloadU8.length} head=${head}`);
  }
	
  const s = this._sync;
  if (!s) return;

  // payloadU8 includes CRC at end
  const computed = computeCrcLikeCSharp(payloadU8);
  const original = getOriginalCrcLE(payloadU8);
  const crcOk = (computed === original);

  // payloadIndex is first 2 bytes (little endian) in your C# bin stream
  const payloadIndex = u16le_at(payloadU8, 0);

  if (!crcOk) {
	  s.lastReply = "NACK";
	  s.nackCrcCount++;
	  this._clearSyncRxBuffers("crc-nack");
	  await this.writeBytes(DATA_NACK);

    if (s.nackCrcCount >= 5) {
      this._abortSync(new Error("Too many CRC failures (NACKCRCcounter>=5)"));
    }

    if (s.onProgress) s.onProgress({ payloadIndex, bytesWritten: s.bytesWritten, crcOk: false });
    return;
  }
  if (this.debugSync) {
    console.log("[sync] writing...", { hasWritable: !!s.writable, payloadLen: payloadU8.length, bytesWrittenBefore: s.bytesWritten });
  }

  // write to bin (either disk stream or memory)
  if (s.writable) {
    await s.writable.write(payloadU8);
  } else {
    s.chunks.push(payloadU8.slice()); // copy
  }
  s.bytesWritten += payloadU8.length;

  // ACK and reset counters (like C# FinishPayload)
  s.lastReply = "ACK";
  s.nackCount = 0;
  s.nackCrcCount = 0;
  await this.writeBytes(DATA_ACK, { withResponse: true });

  if (s.onProgress) s.onProgress({ payloadIndex, bytesWritten: s.bytesWritten, crcOk: true });
}

  
  
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
  try {
    if (this._mode === "logged" && this.debugSync) {
      this._syncRxCount++;
      if (this._syncRxCount <= 25 || (this._syncRxCount % 100) === 0) {
        const head = Array.from(bytes.slice(0, 8)).map(b => b.toString(16).padStart(2,"0")).join(" ");
        console.log(`[sync][rx#${this._syncRxCount}] len=${bytes.length} head=${head}`, {
        newPayload: this._newPayload,
        expectedLen: this._expectedLen,
        bufLen: this._buf?.length ?? 0,
        mode: this._mode
    });
  }
}

	  
    // If you're in logged-sync mode, refresh timeout watchdog on *any* RX
    if (this._mode === "logged" && this._sync) this._sync.lastRxAt = Date.now();

    // Logged sync EOS marker: 0x42 00 00
    if (this._mode === "logged" && bytes.length === 3 && bytes[0] === DATA_EOS_HDR) {
	  if (this.debugSync) console.log("[sync] EOS received (0x42 00 00). Finishing. bytesWritten=", this._sync?.bytesWritten);
      this._resetAssembler();
      this._finishSync();
      return;
    }

    // Streaming special case: 0x4A 00 00 (len=0) seen sometimes
    if (this._mode === "streaming" && bytes.length === 3 && bytes[0] === 0x4A) return;

    // Assemble: first chunk has [hdr][lenLE16] then payload bytes; later chunks are raw continuation.
    if (this._newPayload) {
      if (bytes.length < 3) return;
      const len = u16le(bytes[1], bytes[2]);

      // Sanity guard: if we ever get desynced (dropped/extra byte), we'll start
      // interpreting random bytes as [hdr][lenLE16]. Bail out and resync.
      const MAX_FRAME = 40000; // expected logged frames are ~32k; keep a little headroom
      if (len < 0 || len > MAX_FRAME) {
        console.warn("[rx] Implausible frame length; resyncing", { hdr: bytes[0], len });
        this._resetAssembler();
        if (bytes.length > 1) {
          const rest = bytes.slice(1);
          const feed = () => this._onRx(rest);
          (typeof queueMicrotask === "function") ? queueMicrotask(feed) : Promise.resolve().then(feed);
        }
        return;
      }

      this._expectedLen = len;
      this._buf = new Uint8Array(0);

      if (bytes.length > 3) this._appendBuf(bytes.slice(3));
      this._newPayload = false;
    } else {
      this._appendBuf(bytes);
    }

    if (this._buf.length < this._expectedLen) return;

    const payload = this._buf.slice(0, this._expectedLen);
    const remainder = (this._buf.length > this._expectedLen)
      ? this._buf.slice(this._expectedLen)
      : null;

    this._resetAssembler();

    // If BLE/serial delivered bytes for *next* frame in the same chunk,
    // do NOT drop them—feed them back in.
    if (remainder && remainder.length) {
      const feed = () => this._onRx(remainder);
      (typeof queueMicrotask === "function") ? queueMicrotask(feed) : Promise.resolve().then(feed);
    }

    // --- LOGGED SYNC PAYLOADS ---
    if (this._mode === "logged") {
      // IMPORTANT: _handleLoggedPayload is async; serialize it.
      this._loggedChain = (this._loggedChain ?? Promise.resolve())
        .then(() => this._handleLoggedPayload(payload))
        .catch((e) => this._abortSync(e));
      return;
    }

    // --- STREAMING PAYLOADS ---
    if (this._mode === "streaming") {
      this._handleStreamingPayload(payload);
      return;
    }

    // --- COMMAND / IDLE PAYLOADS (THIS FIXES opcfg TIMEOUT) ---
    const pending = this._pending;
    this._pending = null;

    if (this._mode === "command") this._mode = "idle";

    if (pending) pending.resolve({ payload });
    this.emit("commandPayload", { payload });

  } catch (e) {
    console.warn("[_onRx] error:", e);
    // If logged sync is active, abort cleanly instead of leaving it wedged
    if (this._mode === "logged") this._abortSync(e);
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
  
  _appendStreamBuf(chunk) {
  const merged = new Uint8Array(this._rxStreamBuf.length + chunk.length);
  merged.set(this._rxStreamBuf, 0);
  merged.set(chunk, this._rxStreamBuf.length);
  this._rxStreamBuf = merged;
}

_feedStreamBytes(chunk) {
  if (this._mode === "logged" && this._sync) this._sync.lastRxAt = Date.now();

  this._appendStreamBuf(chunk);

  while (true) {
    if (this._rxStreamBuf.length < 3) return;

    const hdr = this._rxStreamBuf[0];
    const len = (this._rxStreamBuf[1] | (this._rxStreamBuf[2] << 8)) >>> 0;

    // consume header
    if (len === 0) {
      this._rxStreamBuf = this._rxStreamBuf.slice(3);

      // EOS marker for sync
      if (this._mode === "logged" && hdr === DATA_EOS_HDR) {
        if (this.debugSync) console.log("[sync] EOS received (stream parser). Finishing.");
        this._finishSync();
      }
      // streaming keepalive 0x4A 00 00: ignore
      continue;
    }

    if (this._rxStreamBuf.length < 3 + len) return;

    const payload = this._rxStreamBuf.slice(3, 3 + len);
    this._rxStreamBuf = this._rxStreamBuf.slice(3 + len);

    // Now dispatch exactly like your “payload complete” section:
    if (this._mode === "logged") {
      this._loggedChain = (this._loggedChain ?? Promise.resolve())
        .then(() => this._handleLoggedPayload(payload))
        .catch((e) => this._abortSync(e));
      continue;
    }

    if (this._mode === "streaming") {
      this._handleStreamingPayload(payload);
      continue;
    }

    const pending = this._pending;
    this._pending = null;
    if (this._mode === "command") this._mode = "idle";
    if (pending) pending.resolve({ payload });
    this.emit("commandPayload", { payload });
  }
}
_clearSyncRxBuffers(reason = "") {
  // When we decide to NACK / retry, discard any partial RX state.
  // Otherwise the resent packet can be concatenated onto stale bytes and misalign parsing.
  const had = {
    rxStreamBufLen: this._rxStreamBuf?.length ?? 0,
    asmBufLen: this._buf?.length ?? 0,
    expectedLen: this._expectedLen ?? 0,
    newPayload: this._newPayload ?? true
  };

  this._rxStreamBuf = new Uint8Array(0);
  this._resetAssembler();

  if (this.debugSync) {
    console.warn("[sync] cleared RX buffers", { reason, had });
  }
}

  
}


