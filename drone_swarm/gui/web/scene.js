import * as THREE from './vendor/three.module.js';
import { OrbitControls } from './vendor/OrbitControls.js';

// Rendering only. All positions are measured Gazebo poses supplied by the API.
export class SwarmScene {
  constructor(container, reportFPS) {
    this.container = container;
    this.reportFPS = reportFPS;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.15;
    container.appendChild(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color('#071521');
    this.scene.fog = new THREE.FogExp2('#0d2232', 0.006);
    this.camera = new THREE.PerspectiveCamera(45, 1, 0.1, 1600);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(0, -60, 14);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0, 14);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.minDistance = 3;
    this.controls.maxDistance = 900;
    this.controls.maxPolarAngle = Math.PI * 0.499;
    this.controls.update();
    const hemisphere = new THREE.HemisphereLight('#b1d5eb', '#142633', 0.65);
    hemisphere.position.set(0, 0, 1);
    this.scene.add(hemisphere);
    const sunset = new THREE.DirectionalLight('#eec49c', 0.75);
    sunset.position.set(-30, -30, 50);
    this.scene.add(sunset);
    const fill = new THREE.DirectionalLight('#4b91be', 0.3);
    fill.position.set(30, 40, 15);
    this.scene.add(fill);
    this.fleet = new THREE.Group();
    this.fleet.visible = false;
    this.scene.add(this.fleet);
    this.count = 0;
    this.actualPositions = [];
    this.dummy = new THREE.Object3D();
    this.color = new THREE.Color();
    this.glowTexture = this.makeGlow();
    this.coreTexture = this.makeGlow(true);
    this.buildEnvironment();
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.resize();
    this.fpsFrames = 0;
    this.fpsStart = performance.now();
    this.animate = this.animate.bind(this);
    this.frame = requestAnimationFrame(this.animate);
  }

  makeGlow(core = false) {
    const canvas = document.createElement('canvas');
    canvas.width = canvas.height = 64;
    const ctx = canvas.getContext('2d');
    const gradient = ctx.createRadialGradient(32, 32, 0, 32, 32, 32);
    gradient.addColorStop(0, 'rgba(255,255,255,1)');
    if (core) {
      gradient.addColorStop(0.5, 'rgba(255,255,255,1)');
      gradient.addColorStop(0.8, 'rgba(255,255,255,.65)');
    } else {
      gradient.addColorStop(0.16, 'rgba(255,255,255,.95)');
      gradient.addColorStop(0.3, 'rgba(255,255,255,.32)');
      gradient.addColorStop(0.65, 'rgba(255,255,255,.055)');
    }
    gradient.addColorStop(1, 'rgba(255,255,255,0)');
    ctx.fillStyle = gradient;
    ctx.fillRect(0, 0, 64, 64);
    return new THREE.CanvasTexture(canvas);
  }

  buildEnvironment() {
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(1200, 1200),
      new THREE.MeshStandardMaterial({ color: '#0b1c27', roughness: 1, metalness: 0.05 }));
    ground.position.z = -0.03;
    this.scene.add(ground);
    const grid = new THREE.GridHelper(240, 120, '#285260', '#1a3849');
    grid.rotation.x = Math.PI / 2;
    grid.material.transparent = true;
    grid.material.opacity = 0.38;
    this.scene.add(grid);
    const deck = new THREE.Mesh(new THREE.BoxGeometry(38, 38, 0.08),
      new THREE.MeshStandardMaterial({ color: '#102833', roughness: 0.95 }));
    deck.position.z = 0.02;
    this.scene.add(deck);
    const pad = new THREE.Mesh(new THREE.RingGeometry(5.85, 5.92, 96),
      new THREE.MeshBasicMaterial({ color: '#3c9b99', transparent: true, opacity: 0.35, side: THREE.DoubleSide }));
    pad.position.z = 0.07;
    this.scene.add(pad);
    const markMaterial = new THREE.MeshBasicMaterial({ color: '#5d9498', transparent: true, opacity: 0.27 });
    for (const [x, z, width, depth] of [[-1, 0, 0.2, 3.4], [1, 0, 0.2, 3.4], [0, 0, 2, 0.2]]) {
      const mark = new THREE.Mesh(new THREE.BoxGeometry(width, depth, 0.01), markMaterial);
      mark.position.set(x, z, 0.08);
      this.scene.add(mark);
    }
    const city = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1),
      new THREE.MeshStandardMaterial({ color: '#172a38', roughness: 1 }), 27);
    const windows = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1),
      new THREE.MeshBasicMaterial({ color: '#5b7c83', transparent: true, opacity: 0.28 }), 27);
    const object = new THREE.Object3D();
    for (let i = 0; i < 27; i++) {
      const variation = (Math.sin(i * 43.91 + 2.4) + 1) / 2;
      const height = 3 + variation * 12;
      const x = (i - 13) * 8;
      const y = 105 + variation * 19;
      object.position.set(x, y, height / 2);
      object.scale.set(4 + variation * 3, 6, height);
      object.updateMatrix();
      city.setMatrixAt(i, object.matrix);
      object.position.set(x, y - 3.02, height * 0.72);
      object.scale.set(2.5 + variation * 2, 0.01, 0.14);
      object.updateMatrix();
      windows.setMatrixAt(i, object.matrix);
    }
    city.instanceMatrix.needsUpdate = true;
    windows.instanceMatrix.needsUpdate = true;
    this.scene.add(city, windows);
  }

  resize() {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }

  clearFleet() {
    const geometries = new Set();
    const materials = new Set();
    for (const object of [...this.fleet.children]) {
      this.fleet.remove(object);
      if (object.geometry) geometries.add(object.geometry);
      if (object.material) materials.add(object.material);
      if (object.isInstancedMesh) object.dispose();
    }
    geometries.forEach(geometry => geometry.dispose());
    materials.forEach(material => material.dispose());
  }

  allocate(count) {
    this.clearFleet();
    this.count = count;
    const addInstances = (geometry, color, instances, metalness = 0.15) => {
      const mesh = new THREE.InstancedMesh(geometry,
        new THREE.MeshStandardMaterial({ color, roughness: 0.5, metalness }), instances);
      mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
      mesh.frustumCulled = false;
      this.fleet.add(mesh);
      return mesh;
    };
    this.bodies = addInstances(new THREE.BoxGeometry(0.3, 0.22, 0.12), '#c0d6e1', count, 0.4);
    this.arms = addInstances(new THREE.BoxGeometry(0.82, 0.04, 0.035), '#547081', count * 2);
    this.motors = addInstances(new THREE.CylinderGeometry(0.043, 0.043, 0.07, 10).rotateX(Math.PI / 2), '#b3cbd6', count * 4);
    this.rotors = addInstances(new THREE.CylinderGeometry(0.14, 0.14, 0.008, 16).rotateX(Math.PI / 2), '#7fb7bc', count * 4);
    this.pointGeometry = new THREE.BufferGeometry();
    this.pointGeometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(count * 3), 3).setUsage(THREE.DynamicDrawUsage));
    this.pointGeometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(count * 3), 3).setUsage(THREE.DynamicDrawUsage));
    this.glows = new THREE.Points(this.pointGeometry, new THREE.PointsMaterial({
      size: 2.3, map: this.glowTexture, vertexColors: true, transparent: true,
      blending: THREE.AdditiveBlending, depthWrite: false, depthTest: false, opacity: 0.85, toneMapped: false, fog: false,
    }));
    this.lights = new THREE.Points(this.pointGeometry, new THREE.PointsMaterial({
      size: 0.4, map: this.coreTexture, vertexColors: true, transparent: true,
      blending: THREE.AdditiveBlending, depthWrite: false, depthTest: false, toneMapped: false, fog: false,
    }));
    this.glows.frustumCulled = this.lights.frustumCulled = false;
    this.fleet.add(this.glows, this.lights);
  }

  update(positions, colors, connected) {
    const valid = connected && Array.isArray(positions) && positions.length > 0 && positions.length <= 400 &&
      positions.every(p => Array.isArray(p) && p.length === 3 && p.every(Number.isFinite));
    this.fleet.visible = Boolean(valid);
    if (!valid) return;
    this.actualPositions = positions;
    if (positions.length !== this.count) this.allocate(positions.length);
    const pointPositions = this.pointGeometry.attributes.position;
    const pointColors = this.pointGeometry.attributes.color;
    const offsets = [[-0.29, -0.29], [-0.29, 0.29], [0.29, -0.29], [0.29, 0.29]];
    for (let i = 0; i < positions.length; i++) {
      const [x, y, z] = positions[i];
      // Preserve Gazebo world XYZ exactly. The camera and scene use Z-up.
      this.dummy.position.set(x, y, z);
      this.dummy.rotation.set(0, 0, 0);
      this.dummy.updateMatrix();
      this.bodies.setMatrixAt(i, this.dummy.matrix);
      for (let arm = 0; arm < 2; arm++) {
        this.dummy.rotation.z = arm ? -Math.PI / 4 : Math.PI / 4;
        this.dummy.updateMatrix();
        this.arms.setMatrixAt(i * 2 + arm, this.dummy.matrix);
      }
      this.dummy.rotation.z = 0;
      offsets.forEach(([dx, dy], rotor) => {
        this.dummy.position.set(x + dx, y + dy, z + 0.025);
        this.dummy.updateMatrix();
        this.motors.setMatrixAt(i * 4 + rotor, this.dummy.matrix);
        this.dummy.position.z = z + 0.07;
        this.dummy.updateMatrix();
        this.rotors.setMatrixAt(i * 4 + rotor, this.dummy.matrix);
      });
      pointPositions.setXYZ(i, x, y, z - 0.07);
      const rgb = Array.isArray(colors?.[i]) && colors[i].length === 3 && colors[i].every(Number.isFinite) ? colors[i] : [0, 0, 0];
      this.color.setRGB(...rgb.map(value => THREE.MathUtils.clamp(value, 0, 1)), THREE.SRGBColorSpace);
      pointColors.setXYZ(i, this.color.r, this.color.g, this.color.b);
    }
    for (const mesh of [this.bodies, this.arms, this.motors, this.rotors]) mesh.instanceMatrix.needsUpdate = true;
    pointPositions.needsUpdate = pointColors.needsUpdate = true;
  }

  fitConfig(config = {}) {
    const size = Math.max(Number(config.size) || 20, Math.sqrt(Number(config.count) || 5) * (Number(config.min_distance) || 1.5));
    const altitude = Number(config.altitude) || 20;
    const center = new THREE.Vector3(0, 0, Math.max(5, altitude * 0.58));
    const radius = Math.max(15, (altitude + size * 0.65) * 0.6, size * 0.7 / this.camera.aspect);
    this.setCamera(center, radius * 2.6, 'front');
  }

  setView(mode) {
    if (!this.actualPositions.length) return;
    const points = this.actualPositions.map(([x, y, z]) => new THREE.Vector3(x, y, z));
    const box = new THREE.Box3().setFromPoints(points);
    const center = box.getCenter(new THREE.Vector3());
    const dimensions = box.getSize(new THREE.Vector3());
    const visibleHeight = mode === 'top' ? Math.max(dimensions.y, dimensions.x / this.camera.aspect) :
      Math.max(dimensions.z, dimensions.x / this.camera.aspect, dimensions.y * 0.65);
    const distance = Math.max(10, visibleHeight * 0.75 / Math.tan(THREE.MathUtils.degToRad(this.camera.fov / 2)));
    this.setCamera(center, distance, mode);
  }

  setCamera(center, distance, mode) {
    const direction = mode === 'fit' ? this.camera.position.clone().sub(this.controls.target).normalize() :
      mode === 'top' ? new THREE.Vector3(0, -0.001, 1) : new THREE.Vector3(0, -1, 0.035);
    this.controls.target.copy(center);
    this.camera.position.copy(center).addScaledVector(direction.normalize(), distance);
    this.controls.update();
  }

  animate(now) {
    this.frame = requestAnimationFrame(this.animate);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this.fpsFrames++;
    if (now - this.fpsStart >= 1000) {
      this.reportFPS(this.fpsFrames * 1000 / (now - this.fpsStart));
      this.fpsFrames = 0;
      this.fpsStart = now;
    }
  }

  dispose() {
    cancelAnimationFrame(this.frame);
    this.resizeObserver.disconnect();
    this.controls.dispose();
    this.clearFleet();
    const geometries = new Set();
    const materials = new Set();
    this.scene.traverse(object => {
      if (object.geometry) geometries.add(object.geometry);
      if (object.material) materials.add(object.material);
    });
    geometries.forEach(geometry => geometry.dispose());
    materials.forEach(material => material.dispose());
    this.glowTexture.dispose();
    this.coreTexture.dispose();
    this.renderer.dispose();
  }
}
