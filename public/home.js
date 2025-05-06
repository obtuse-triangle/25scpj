// Constants
const THRESHOLD = 0.2;
const FRICTION = 0.9;
const C_d = 1.0;
const rho = 1.225;
const A = 0.045;
const MASS = 1.0;
const MAX_SAMPLES = 20;
const INITIALIZATION_DELAY = 2000; // 2 seconds delay

// Calibration Layer
class CalibrationManager {
  constructor(maxSamples = MAX_SAMPLES) {
    this.samples = [];
    this.maxSamples = maxSamples;
    this.gravityBias = { x: 0, y: 0, z: 0 };
    this.isCalibrated = false;
  }

  addSample(sample) {
    this.samples.push(sample);
    if (this.samples.length > this.maxSamples) {
      this.samples.shift();
    }
  }

  getMeanAndVariance() {
    const n = this.samples.length;
    if (n === 0) return { mean: { x: 0, y: 0, z: 0 }, variance: 0 };

    const mean = this.samples.reduce(
      (acc, s) => ({
        x: acc.x + s.x / n,
        y: acc.y + s.y / n,
        z: acc.z + s.z / n,
      }),
      { x: 0, y: 0, z: 0 }
    );

    const variance = this.samples.reduce(
      (acc, s) => acc + ((s.x - mean.x) ** 2 + (s.y - mean.y) ** 2 + (s.z - mean.z) ** 2) / (3 * n),
      0
    );

    return { mean, variance };
  }

  isStationary(threshold = 0.8) {
    const { variance } = this.getMeanAndVariance();
    return variance < threshold;
  }

  updateGravityBias() {
    if (this.isStationary()) {
      const { mean } = this.getMeanAndVariance();
      this.gravityBias = mean;
      this.isCalibrated = true;
      return true;
    }
    return false;
  }

  resetSamples() {
    this.samples.length = 0;
  }
}

// Physics Layer
class PhysicsEngine {
  constructor(mass = MASS) {
    this.mass = mass;
    this.velocity = { x: 0, y: 0, z: 0 };
    this.position = { x: 0, y: 0, z: 0 };
    this.totalEnergy = 0;
  }

  filterNoise(a) {
    return Math.abs(a) < THRESHOLD ? 0 : a;
  }

  update(acc, dt) {
    // Apply noise filter
    const ax = this.filterNoise(acc.x);
    const ay = this.filterNoise(acc.y);
    const az = this.filterNoise(acc.z);

    // Update velocity
    this.velocity.x += ax * dt;
    this.velocity.y += ay * dt;
    this.velocity.z += az * dt;

    // Drag force
    const vMag = Math.sqrt(this.velocity.x ** 2 + this.velocity.y ** 2 + this.velocity.z ** 2);
    if (vMag > 0.1) {
      const dragForce = 0.5 * C_d * rho * A * vMag ** 2;
      const dragAcc = dragForce / this.mass;

      this.velocity.x -= (this.velocity.x / vMag) * dragAcc * dt;
      this.velocity.y -= (this.velocity.y / vMag) * dragAcc * dt;
      this.velocity.z -= (this.velocity.z / vMag) * dragAcc * dt;

      this.totalEnergy += dragForce * dt;
    }

    // Friction
    this.velocity.x *= FRICTION;
    this.velocity.y *= FRICTION;
    this.velocity.z *= FRICTION;

    // Update position
    const dx = this.velocity.x * dt;
    const dy = this.velocity.y * dt;
    const dz = this.velocity.z * dt;
    this.position.x += dx;
    this.position.y += dy;
    this.position.z += dz;

    // Distance
    const distance = Math.sqrt(dx ** 2 + dy ** 2 + dz ** 2);

    // Force and energy
    const avgAcc = Math.sqrt(ax ** 2 + ay ** 2 + az ** 2);
    const force = this.mass * avgAcc;
    const energy = force * distance;
    this.totalEnergy += energy;

    return {
      velocity: { ...this.velocity },
      gravityBias: null, // to be filled by upper layer
      totalEnergy: this.totalEnergy,
      distance,
      force,
      avgAcc,
      isStationary:
        this.velocity.x.toFixed(3) == 0 &&
        this.velocity.y.toFixed(3) == 0 &&
        this.velocity.z.toFixed(3) == 0,
    };
  }
}

// UI Layer
class UIManager {
  static update(data, gravityBias, dt, count) {
    document.querySelector("#motionData").value =
      `속도: (${data.velocity.x.toFixed(3)}, ${data.velocity.y.toFixed(
        3
      )}, ${data.velocity.z.toFixed(3)})\n` +
      `중력 보정값: (${gravityBias.x.toFixed(2)}, ${gravityBias.y.toFixed(
        2
      )}, ${gravityBias.z.toFixed(2)})\n` +
      `총 에너지: ${data.totalEnergy.toFixed(3)} J\n` +
      `거리: ${data.distance.toFixed(3)} m\n` +
      `힘: ${data.force.toFixed(3)} N\n` +
      `평균 가속도: ${data.avgAcc.toFixed(3)} m/s²\n` +
      `정지 상태: ${data.isStationary ? "🟢 YES" : "🔴 NO"}\n`; //+
    //`단위 시간: ${dt.toFixed(3)} s\n`/  / +
    // `count: ${count}`;
  }
}

// Sensor Layer
class SensorManager {
  constructor() {
    this.calibrationManager = new CalibrationManager();
    this.physicsEngine = new PhysicsEngine();
    this.lastTimestamp = null;
    this.count = 0;
    this.isInitialized = false;
    this.init();
  }

  init() {
    window.addEventListener("devicemotion", (event) => this.handleMotion(event));

    // Wait for initialization and calibration
    setTimeout(() => {
      if (this.calibrationManager.isCalibrated) {
        this.isInitialized = true;
        // Dispatch a custom event when initialization is complete
        const initEvent = new CustomEvent("motionSensorInitialized", {
          detail: {
            gravityBias: this.calibrationManager.gravityBias,
            message: "Motion sensor calibrated and ready",
          },
        });
        window.dispatchEvent(initEvent);
        console.log("Motion sensor initialized and calibrated");
      } else {
        console.log("Motion sensor calibration still in progress");
      }
    }, INITIALIZATION_DELAY);
  }

  handleMotion(event) {
    const acc = event.accelerationIncludingGravity;
    if (!acc || acc.x === null || acc.y === null || acc.z === null) return;

    this.calibrationManager.addSample({ x: acc.x, y: acc.y, z: acc.z });
    if (this.count === MAX_SAMPLES) {
      this.calibrationManager.updateGravityBias();
      this.count = 0;
      this.calibrationManager.resetSamples();
    }
    this.count++;

    // Gravity bias correction
    const gravityBias = this.calibrationManager.gravityBias;
    const accCorrected = {
      x: acc.x - gravityBias.x,
      y: acc.y - gravityBias.y,
      z: acc.z - gravityBias.z,
    };

    const timestamp = event.timeStamp;
    if (this.lastTimestamp !== null) {
      const dt = (timestamp - this.lastTimestamp) / 1000;
      const data = this.physicsEngine.update(accCorrected, dt);
      data.gravityBias = gravityBias;
      UIManager.update(data, gravityBias, dt, this.count);
    }
    this.lastTimestamp = timestamp;
  }
}

// App Entry
new SensorManager();

// Example of how to use the custom event
window.addEventListener("motionSensorInitialized", (event) => {
  // You can write your custom event handling code here
  console.log("Event triggered:", event.detail.message);
  // Your code will be executed here after calibration
  document.querySelector("section.fixed").hidden = true;
});
