import * as THREE from 'three'
import {Axes} from "./Axes.js";
import {ensureVector3, integrateTwistStepwise} from "./linAlg.js";

/**
 * Class that renders a twist in 3D.
 */


export class TwistViz {

    /**
     * Constructor for the TwistViz class.
     *
     * @param {Object} params - Configuration parameters for the TwistViz.
     * @param {HTMLElement} params.container - HTML container for the 3D visualization.
     * @param {HTMLCanvasElement} params.canvas - Canvas for rendering the 3D visualization.
     * @param {THREE.Vector3} params.targetPosition - The target point for the camera
     * @param {THREE.Matrix4} params.camera_pose - Initial camera pose for the view.
     * @param {number} params.scale - Percentage of container size to occupy
     */


    constructor({container: container, canvas: canvas, target_position: targetPosition, camera_pose: cameraPose, scale: scale = 1.0}) {
        this.container = container
        this.scale = scale
        this.scene = new THREE.Scene();
        this.anchorMarker = new Axes({size: .5, thickness: 0.025, opacity: 0.25});
        this.anchorMarker.setColors(0xff0000, 0x00ff00, 0x000ff);
        this.movingMarker = new Axes({size: 1});
        this.movingMarker.setColors(0xff0000, 0x00ff00, 0x000ff);
        this.scene.add(this.movingMarker);
        this.scene.add(this.anchorMarker);

        this.twistTrace = new THREE.Line(
            new THREE.BufferGeometry(),
            new THREE.LineBasicMaterial({color: 0x999999, linewidth: 5}));

        this.scene.add(this.twistTrace)
        this._animationRequest = null;

        this.camera = new THREE.PerspectiveCamera(50, container.innerWidth / container.innerHeight, 0.6, 1000);
        this.camera.up.set(0, 0, 1)
        this.configureForView(targetPosition, cameraPose)
        this.renderer = new THREE.WebGLRenderer({antialias: true, alpha: true, canvas: canvas});
        this.renderer.setClearColor("#FFFFFF", 0)
        this.renderer.setSize(container.clientWidth, container.clientHeight);
        {
            const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
            const directionalLight = new THREE.DirectionalLight(0xffffff, 1.5);
            directionalLight.position.x = -2
            directionalLight.position.y = -2
            directionalLight.position.z = 2

            this.scene.add(ambientLight);
            this.scene.add(directionalLight)
        }
        {
            const color = 0xFFFFFF
            const near = 3
            const far = 8
            this.scene.fog = new THREE.Fog(color, near, far);
        }

        this._resizeObserver = new ResizeObserver(this._elementResized.bind(this)).observe(container);

    }

    /**
     * Begins the visualization by resetting the view and starting the rendering loop.
     */

    begin() {
        this.reset();
        this._render();
    }

    /**
     * Stops the visualization rendering loop.
     */

    stop() {
        cancelAnimationFrame(this._animationRequest);
    }

    /**
     * Sets the twist for the visualization and updates the position and orientation of markers.
     *
     * @param {Array} twist - A 2-element array containing linear and angular twists.
     */


    setTwist(twist) {
        const twistTracePoints = integrateTwistStepwise(twist, 1, 100)
        this.twistTrace.geometry.setFromPoints(twistTracePoints)

        let marker = this.movingMarker
        const finalPoint = twistTracePoints[twistTracePoints.length - 1]
        marker.position.copy(finalPoint)
        marker.rotation.copy(new THREE.Euler().setFromVector3(ensureVector3(twist[1])))
    }

    /**
     * Sets the orientation for the visualization markers.
     *
     * @param {THREE.Matrix3} rotation - The rotation matrix to set the marker's orientation.
     */
    setMarkerOrientation(rotation) {
        const rotMatrix4 = new THREE.Matrix4().setFromMatrix3(rotation)
        this.anchorMarker.setRotationFromMatrix(rotMatrix4)
        this.movingMarker.setRotationFromMatrix(rotMatrix4)
    }

    /**
     * Render function that draws the current state of the visualization and schedules the next frame.
     */

    _render() {
        this.renderer.render(this.scene, this.camera);
        this._animationRequest = requestAnimationFrame(this._render.bind(this));
    }

    /**
     * Callback to handle when the container element is resized.
     */

    _elementResized(_) {
        this.renderer.setSize(this.container.clientWidth * this.scale, this.container.clientHeight * this.scale);
        this.camera.aspect = this.container.clientWidth / this.container.clientHeight;
        this.camera.updateProjectionMatrix();
    }

    /**
     * Sets the camera to its initial state.
     */

    setCameraToInitialState() {
        this.camera.position.set(-.5, -3, 3);
        this.camera.lookAt(0, 0, 0);
    }

    /**
     * Configures the view based on the target and camera poses.
     *
     * @param {THREE.Vector3|null} targetPosition - The target to point the camera at.
     * @param {THREE.Matrix4} cameraPose - Camera pose for the view.
     */

    configureForView(targetPosition, cameraPose) {
        let position = new THREE.Vector3().setFromMatrixPosition(cameraPose)
        this.camera.position.copy(position)
        if (!targetPosition) {
            // Look at the origin by default
            targetPosition = new THREE.Vector3(0,0,0)
        }
        this.camera.lookAt(targetPosition.x, targetPosition.y, targetPosition.z)
    }

    /**
     * Resets the moving marker to its initial state.
     */

    reset() {
        this.movingMarker.position.x = 0;
        this.movingMarker.position.y = 0;
        this.movingMarker.position.z = 0;
        this.movingMarker.rotation.x = 0;
        this.movingMarker.rotation.y = 0;
        this.movingMarker.rotation.z = 0;
    }

}
