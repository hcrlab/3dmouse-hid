import * as THREE from 'three'
import {Axes} from "./Axes.js";
import {ensureVector3, integrateTwistStepwise} from "./linAlg.js";


export class TwistViz {

    constructor(container) {
        this.container = container
        this.scene = new THREE.Scene();
        this.anchorMarker = new Axes({size: .5, thickness: 0.025, opacity: 0.25});
        this.anchorMarker.setColors(0xff0000, 0x00ff00, 0x000ff);
        this.movingMarker = new Axes({size: 1});
        this.movingMarker.setColors(0xff0000, 0x00ff00, 0x000ff);
        this.scene.add(this.movingMarker);
        this.scene.add(this.anchorMarker);

        this.twistTrace = new THREE.Line( 
            new THREE.BufferGeometry(), 
            new THREE.LineBasicMaterial( { color: 0x999999, linewidth: 5 } ) );

        this.scene.add(this.twistTrace)
        this._animationRequest = null;

        this.camera = new THREE.PerspectiveCamera(50, container.innerWidth / container.innerHeight, 0.6, 1000);
        this.camera.up.set(0,0,1)
        this.renderer = new THREE.WebGLRenderer({antialias: true, alpha: true});
        this.renderer.setClearColor("#FFFFFF", 0)
        container.appendChild(this.renderer.domElement)
        this.renderer.setSize(container.clientWidth, container.clientHeight);
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
        const directionalLight = new THREE.DirectionalLight( 0xffffff, 1.5);
        directionalLight.position.x = -2
        directionalLight.position.y = -2
        directionalLight.position.z = 2

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

    setTwist(twist) {
        const twistTracePoints = integrateTwistStepwise(twist, 1, 100)
        this.twistTrace.geometry.setFromPoints(twistTracePoints)

        let marker = this.movingMarker
        const finalPoint = twistTracePoints[twistTracePoints.length - 1]
        marker.position.copy(finalPoint)
        marker.rotation.copy(new THREE.Euler().setFromVector3(ensureVector3(twist[1])))
    }

    setMarkerOrientation(rotation) {
        const rotMatrix4 = new THREE.Matrix4().setFromMatrix3(rotation)
        this.anchorMarker.setRotationFromMatrix(rotMatrix4)
        this.movingMarker.setRotationFromMatrix(rotMatrix4)
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

    configureForView(targetPose, cameraPose) {
        let position = new THREE.Vector3().setFromMatrixPosition(cameraPose)
        this.camera.position.copy(position)
        this.camera.lookAt(0,0,0)
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
