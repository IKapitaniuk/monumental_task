import { GUI } from 'three/examples/jsm/libs/lil-gui.module.min.js';
import {
  IncomingMessage,
  ConfigurationRequest,
  StateMessage,
  OutcomingMessage,
} from './message';
import { Scene3D } from './scene';
import { RobotConfiguration } from './robot_configuration';
import { Robot } from './robot';
import { RobotGUI } from './robot_gui';
import { RobotController } from './robot_controller';
import * as THREE from 'three';

export class Application {
  private ws: WebSocket | null;
  private to_connect: boolean;
  private url: string;
  private gui: GUI;
  private scene: Scene3D;
  private robot: Robot | null;
  private robot_gui: RobotGUI | null;
  private state: StateMessage | null;
  private target: THREE.Mesh;

  constructor(gui: GUI, scene: Scene3D) {
    this.ws = null;
    this.to_connect = false;
    this.url = '';
    this.state = null;

    this.gui = gui;
    this.scene = scene;
    this.robot = null;
    this.robot_gui = null;

    this.set_network_gui();
    // Target
    const material = new THREE.MeshStandardMaterial();
    material.color = new THREE.Color(1, 0, 0);
    const sphereGeometry = new THREE.SphereGeometry(0.03);
    this.target = new THREE.Mesh(sphereGeometry, material);
    this.target.add(new THREE.AxesHelper(0.5));
    this.target.visible = false;
    this.scene.add(this.target);
  }

  private set_network_gui() {
    const url = { ip_address: 'localhost', port: 8081, connect: () => {} };
    const net = this.gui.addFolder('Network Connection');
    const addr = net.add(url, 'ip_address');
    const port = net.add(url, 'port');
    const btn = net.add(url, 'connect').name('Connect');

    url.connect = () => {
      if (!this.is_connected()) {
        const ws_address = 'ws://' + url.ip_address + ':' + url.port;
        this.connect(ws_address);
        addr.disable();
        port.disable();
        btn.name('Disconnect');
      } else {
        this.disconnect();
        addr.enable();
        port.enable();
        btn.name('Connect');
      }
    };
  }

  private robot_setup(config: RobotConfiguration) {
    if (this.robot !== null) {
      this.scene.remove(this.robot.get_root());
      this.robot_gui?.remove();
      this.target.visible = false;
    }
    this.robot = new Robot('Robot 1', config);
    this.scene.add(this.robot.get_root());
    const controller = new RobotController(
      this.robot,
      (msg: OutcomingMessage) => {
        this.send(msg);
      }
    );
    this.robot_gui = new RobotGUI(
      this.robot,
      controller,
      this.target,
      this.gui
    );
  }

  public connect(url: string) {
    this.to_connect = true;
    this.url = url;
    this.reconnect();
  }

  public disconnect() {
    this.to_connect = false;
    this.ws?.close();
  }

  public get_state(): StateMessage | null {
    return this.state;
  }

  public is_connected(): boolean {
    return this.to_connect;
  }

  private reconnect() {
    this.ws = new WebSocket(this.url);
    this.ws.onmessage = (event) => {
      this.dispatch(event.data);
    };
    this.ws.onopen = () => {
      console.log('connected to server');
      const msg: ConfigurationRequest = {
        msg_id: 'configuration_request',
      };
      this.send(msg);
    };
    this.ws.onerror = (err) => {
      console.log('Error: ', err);
      this.ws?.close();
    };
    this.ws.onclose = () => {
      console.log('disconnected');
      if (this.to_connect) {
        setTimeout(() => {
          this.reconnect();
        }, 1000);
      }
    };
  }

  private send(msg: OutcomingMessage) {
    const raw_msg = JSON.stringify(msg) + '\n'; // Note! Boost::Beast require \n to indentify the end of the message
    this.ws?.send(raw_msg);
  }

  private dispatch(raw_msg: string) {
    const msg: IncomingMessage = JSON.parse(raw_msg);

    if (msg.msg_id === 'state') {
      this.state = msg;
      this.robot?.set_state(this.state);
    } else if (msg.msg_id === 'configuration_update') {
      const config: RobotConfiguration = JSON.parse(msg.configuration);
      this.robot_setup(config);
    }
  }
}
