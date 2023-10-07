import * as THREE from 'three'
import {Axes} from "./Axes.js";

function integrateTwistStepwise(twist, time, steps) {
    const [linear, angular] = [new THREE.Vector3(...twist[0]), new THREE.Vector3(...twist[1])]
    const dt = time / steps
    let points = [new THREE.Vector3(0,0,0)]
    let stepRotation = angular.clone().multiplyScalar(dt)
    let linearStep = linear.clone().multiplyScalar(dt)
    for (let i = 1; i < steps; i++) {
        // New point is the previous rotated then translated
        const newPoint = points[i - 1].clone()
        newPoint.applyEuler(new THREE.Euler(...stepRotation)).add(linearStep)
        points.push(newPoint)
    }
    return points
}

export class TwistViz {

    constructor(container) {
        this.container = container
        this.exageration = 1.
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
        this.renderer = new THREE.WebGLRenderer({antialias: true});
        this.renderer.setClearColor("#FFFFFF"); // Set background colour
        container.appendChild(this.renderer.domElement); // Add renderer to HTML as a
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

    setTwist([linear, angular]) {
        const twistTracePoints = integrateTwistStepwise([linear, angular], 1, 100)
        this.twistTrace.geometry.setFromPoints(twistTracePoints)

        let marker = this.movingMarker
        const finalPoint = twistTracePoints[twistTracePoints.length - 1]
        marker.position.copy(finalPoint)

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
