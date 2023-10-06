import * as THREE from 'three'
import {AxesHelper} from 'three'
import {Axes} from "./Axes.js";

export class TwistViz {

    constructor(container) {
        this.container = container
        this.exageration = 1.
        this.scene = new THREE.Scene();
        this.anchorMarker = new AxesHelper(.5);
        this.anchorMarker.setColors(0xaa0000, 0x00aa00, 0x000aa);
        this.movingMarker = new Axes({size: 1});
        this.movingMarker.setColors(0xff0000, 0x00ff00, 0x000ff);
        this.scene.add(this.movingMarker);
        this.scene.add(this.anchorMarker);
        this._animationRequest = null;
        // Camera
        this.camera = new THREE.PerspectiveCamera(50, container.innerWidth / container.innerHeight, 0.6, 1000);
        this.camera.up.set(0,0,1)
        // Renderer
        this.renderer = new THREE.WebGLRenderer({antialias: true});
        this.renderer.setClearColor("#FFFFFF"); // Set background colour
        container.appendChild(this.renderer.domElement); // Add renderer to HTML as a
        this.renderer.setSize(container.clientWidth, container.clientHeight);
        // Add ambient light
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
        const directionalLight = new THREE.DirectionalLight( 0xffffff, 1.5);
        directionalLight.position.x = -10
        directionalLight.position.z = 10

        this.scene.add(ambientLight);
        this.scene.add(directionalLight)

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

    setTwist([linear, angular]) {
        let marker = this.movingMarker
        marker.position.x = linear[0] * this.exageration
        marker.position.y = linear[1] * this.exageration
        marker.position.z = linear[2] * this.exageration

        marker.rotation.x = angular[0] 
        marker.rotation.y = angular[1]
        marker.rotation.z = angular[2]
    }

    _render() {
        this.renderer.render(this.scene, this.camera);
        this._animationRequest = requestAnimationFrame(this._render.bind(this));
    }

    _elementResized(_) {
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.camera.aspect = this.container.clientWidth / this.container.clientHeight; 
        this.camera.updateProjectionMatrix();
    }

    setCameraToInitialState() {
        this.camera.position.set(-.5, -3, 3);
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
