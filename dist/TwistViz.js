import * as THREE from "https://unpkg.com/three@0.156.1/build/three.module.js"
import {AxesHelper} from "https://unpkg.com/three@0.156.1/build/three.module.js"

export class TwistViz {

    constructor(container) {
        this.container = container
        this.scene = new THREE.Scene();
        this.anchorMarker = new AxesHelper(5);
        this.anchorMarker.setColors(0xaa0000, 0x00aa00, 0x000aa);
        this.movingMarker = new AxesHelper(20);
        this.movingMarker.setColors(0xff0000, 0x00ff00, 0x000ff);
        this.scene.add(this.movingMarker);
        this.scene.add(this.anchorMarker);
        this._animationRequest = null;
        // Camera
        this.camera = new THREE.PerspectiveCamera(100, container.innerWidth / container.innerHeight, 0.6, 1000);
        // Renderer
        this.renderer = new THREE.WebGLRenderer({antialias: true});
        this.renderer.setClearColor("#233143"); // Set background colour
        container.appendChild(this.renderer.domElement); // Add renderer to HTML as a
        this.renderer.setSize(container.clientWidth, container.clientHeight);
        // Add ambient light
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
        this.scene.add(ambientLight);

        this._resizeObserver = new ResizeObserver(this._elementResized.bind(this)).observe(container);

    }

    begin() {
        this.setCameraToInitialState();
        this.reset();
        this._render();
    }

    stop() {
        cancelAnimationFrame(this._animationRequest);
    }

    setTwist(twist) {
        let marker = this.movingMarker
        marker.position.x = -twist[0] * 6 
        marker.position.z = -twist[1] * 6
        marker.position.y = -twist[2] * 4

        marker.rotation.x = -twist[3] 
        marker.rotation.z = -twist[4] 
        marker.rotation.y = -twist[5]
    }

    _render() {
        this.renderer.setClearColor("#000000"); // Black background
        this.renderer.render(this.scene, this.camera);
        this._animationRequest = requestAnimationFrame(this._render.bind(this));
    }

    _elementResized(_) {
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.camera.aspect = this.container.clientWidth / this.container.clientHeight; 
        this.camera.updateProjectionMatrix();
    }

    setCameraToInitialState() {
        this.camera.position.set(30, 5, 2);
        this.camera.lookAt(0, 0, 0);
    }

    reset(){
        this.movingMarker.position.x=0;
        this.movingMarker.position.y=0;
        this.movingMarker.position.z=0;
        this.movingMarker.rotation.x =0;
        this.movingMarker.rotation.y =0;
        this.movingMarker.rotation.z =0;
    
    }

}
