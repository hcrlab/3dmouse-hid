import * as THREE from "https://unpkg.com/three@0.126.1/build/three.module.js"
import { BufferGeometryUtils } from "https://unpkg.com/three@0.126.1/examples/jsm/utils/BufferGeometryUtils.js";
// import { handleInputReport } from '../dist/space-driver.js';
import * as SpaceDriver from '../dist/space-driver.js';

SpaceDriver.configureCallback(processInput)

// Scene
const scene = new THREE.Scene();
function setCameraToInitialState() {
    camera.position.set(30, 5, 2);
    camera.lookAt(0, 0, 0);
}
function frametoinistialstate(){
    mergedMesh.position.x=0;
    mergedMesh.position.y=0;
    mergedMesh.position.z=0;
    mergedMesh.rotation.x =0;
    mergedMesh.rotation.y =0;
    mergedMesh.rotation.z =0;

}

// function someFunction() {
//     if (SpaceDriver.window.latestEvent) {
//         // Use window.latestEvent here
//         const report = SpaceDriver.handleInputReport(e);
//         processInput(report);
//     }
//  }
 
// someFunction()
//Red X-axis
const curveX = new THREE.LineCurve3(
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(10, 0, 0)
);
const geometryX = new THREE.TubeBufferGeometry(curveX, 64, 0.1, 8, false);
const materialX = new THREE.MeshLambertMaterial({color: 0xff0000});
const meshX = new THREE.Mesh(geometryX, materialX);


// Green Y-axis
const curveY = new THREE.LineCurve3(
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(0, 10, 0)
);
const geometryY = new THREE.TubeBufferGeometry(curveY, 64, 0.1, 8, false);
const materialY = new THREE.MeshLambertMaterial({color: 0x00ff00});
const meshY = new THREE.Mesh(geometryY, materialY);


// // Blue Z-axis
const curveZ = new THREE.LineCurve3(
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(0, 0, -10)
);
const geometryZ = new THREE.TubeBufferGeometry(curveZ, 64, 0.1, 8, false);
const materialZ = new THREE.MeshLambertMaterial({color: 0x0000ff});
const meshZ = new THREE.Mesh(geometryZ, materialZ);




function setVertexColors(geometry, color) {
    const colors = [];
    for (let i = 0; i < geometry.attributes.position.count; i++) {
        colors.push(color.r, color.g, color.b);
    }
    geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
}

// Set vertex colors for each geometry
setVertexColors(geometryX, new THREE.Color(0xff0000));
setVertexColors(geometryY, new THREE.Color(0x0000ff));
setVertexColors(geometryZ, new THREE.Color(0x00ff00));

// Merge the geometries
const mergedGeometry = BufferGeometryUtils.mergeBufferGeometries([geometryX, geometryY, geometryZ]);

// Use a material that respects vertex colors
const mergedMaterial = new THREE.MeshBasicMaterial({ vertexColors: true });

const mergedMesh = new THREE.Mesh(mergedGeometry, mergedMaterial);
scene.add(mergedMesh);

// Camera
const camera = new THREE.PerspectiveCamera(100, window.innerWidth / window.innerHeight, 0.6, 2000);

function softmax(Tx, Ty, Tz) {
    let sum = (Tx) + (Ty) + (Tz);

    let wx = (Tx) / sum;
    let wy = (Ty) / sum;
    let wz = (Tz) / sum;

    console.log("xxxxxx" +wx);  // Now, this will correctly log the value of wx

    return {
        wx: wx,
        wy: wy,
        wz: wz
    };
}

function softmax_r(Rx, Ry, Rz) {
    let sum = (Rx) + (Ry) + (Rz);

    return {
        wx: (Rx) / sum,
        wy: (Ry) / sum,
        wz: (Rz) / sum
    };
}



function processInput(report) {
    if (report.Tx === 0 && report.Ty === 0 && report.Tz === 0) {
        frametoinistialstate();
    } else {
        let { wx, wy, wz } = softmax(report.Tx, report.Ty, report.Tz);
        console.log(`XX: ${wx}, YY: ${wy}, ZZ: ${wz}`);
        
        mergedMesh.position.x = -report.Tx * wx / 200;
        mergedMesh.position.z = -report.Ty * wy / 200;
        mergedMesh.position.y = -report.Tz * wz / 200;
        console.log("xx"+mergedMesh.position); // To log all positions at once
    }

    if (report.Rx === 0 && report.Ry === 0 && report.Rz === 0) {
        frametoinistialstate();
    } else {
        let { wx, wy, wz } = softmax_r(report.Rx, report.Ry, report.Rz);
        console.log(`XX: ${wx}, YY: ${wy}, ZZ: ${wz}`);
        
        mergedMesh.rotation.x = -report.Rx * wx / 450;
        mergedMesh.rotation.z = -report.Ry * wy / 450;
        mergedMesh.rotation.y = -report.Rz * wz / 450;
        console.log(mergedMesh.rotation); // To log all rotations at once
    }
}

// Set initial camera state when starting
setCameraToInitialState();


// const axesHelper = new THREE.AxesHelper(10);  // 5 is the size; adjust as needed
// scene.add(axesHelper);


// Renderer
const renderer = new THREE.WebGLRenderer({antialias: true});
renderer.setClearColor("#233143"); // Set background colour
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement); // Add renderer to HTML as a canvas element
// Make Canvas Responsive
window.addEventListener('resize', () => {
    renderer.setSize(window.innerWidth, window.innerHeight); // Update size
    camera.aspect = window.innerWidth / window.innerHeight; // Update aspect ratio
    camera.updateProjectionMatrix(); // Apply changes
})


// Add ambient light
const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
scene.add(ambientLight);


// Rendering
const rendering = function() {
    requestAnimationFrame(rendering);
    renderer.setClearColor("#000000"); // Black background
    renderer.render(scene, camera);
}
rendering();
