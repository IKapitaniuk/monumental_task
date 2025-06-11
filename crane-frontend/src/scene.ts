import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import Stats from 'three/addons/libs/stats.module.js';

export class Scene3D {
  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;
  private renderer: THREE.WebGLRenderer;
  private stats: Stats;

  constructor() {
    THREE.Object3D.DEFAULT_UP = new THREE.Vector3(0, 0, 1);
    this.scene = new THREE.Scene();
    this.scene.add(new THREE.AxesHelper(5));
    const grid = new THREE.GridHelper();
    grid.rotateX(Math.PI / 2);
    this.scene.add(grid);
    this.camera = new THREE.PerspectiveCamera(
      75,
      window.innerWidth / window.innerHeight,
      0.1,
      1000
    );
    this.camera.position.set(-2, 4, 5);
    this.renderer = new THREE.WebGLRenderer();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    document.body.appendChild(this.renderer.domElement);
    new OrbitControls(this.camera, this.renderer.domElement);

    window.addEventListener('resize', () => {
      this.camera.aspect = window.innerWidth / window.innerHeight;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(window.innerWidth, window.innerHeight);
    });

    this.stats = new Stats();
    document.body.appendChild(this.stats.dom);

    const data = { color: 0x00ff00, lightColor: 0xffffff };
    const ambientLight = new THREE.AmbientLight(data.lightColor, Math.PI);
    ambientLight.visible = true;
    this.scene.add(ambientLight);
  }

  public add(obj: THREE.Object3D) {
    this.scene.add(obj);
  }

  public remove(obj: THREE.Object3D) {
    obj.clear();
    this.scene.remove(obj);
  }

  public animate() {
    this.renderer.render(this.scene, this.camera);
    this.stats.update();
  }
}
