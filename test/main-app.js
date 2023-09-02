import * as THREE from "https://unpkg.com/three@0.126.1/build/three.module.js"
import { BufferGeometryUtils } from "https://unpkg.com/three@0.126.1/examples/jsm/utils/BufferGeometryUtils.js";
// import { handleInputReport } from '../dist/space-driver.js';
import * as SpaceDriver from '../dist/space-driver.js';
import * as Scene from '../dist/scene.js';
import DataManager from '../dist/dataManager.js';
SpaceDriver.configureCallback(processInput)





// // Scene
// const scene = new THREE.Scene();
// function setCameraToInitialState() {
//     camera.position.set(30, 5, 2);
//     camera.lookAt(0, 0, 0);
// }
// function frametoinistialstate(){
//     mergedMesh.position.x=0;
//     mergedMesh.position.y=0;
//     mergedMesh.position.z=0;
//     mergedMesh.rotation.x =0;
//     mergedMesh.rotation.y =0;
//     mergedMesh.rotation.z =0;

// }


// //Red X-axis
// const curveX = new THREE.LineCurve3(
//     new THREE.Vector3(0, 0, 0),
//     new THREE.Vector3(10, 0, 0)
// );
// const geometryX = new THREE.TubeBufferGeometry(curveX, 64, 0.1, 8, false);
// const materialX = new THREE.MeshLambertMaterial({color: 0xff0000});
// const meshX = new THREE.Mesh(geometryX, materialX);


// // Green Y-axis
// const curveY = new THREE.LineCurve3(
//     new THREE.Vector3(0, 0, 0),
//     new THREE.Vector3(0, 10, 0)
// );
// const geometryY = new THREE.TubeBufferGeometry(curveY, 64, 0.1, 8, false);
// const materialY = new THREE.MeshLambertMaterial({color: 0x00ff00});
// const meshY = new THREE.Mesh(geometryY, materialY);


// // // Blue Z-axis
// const curveZ = new THREE.LineCurve3(
//     new THREE.Vector3(0, 0, 0),
//     new THREE.Vector3(0, 0, -10)
// );
// const geometryZ = new THREE.TubeBufferGeometry(curveZ, 64, 0.1, 8, false);
// const materialZ = new THREE.MeshLambertMaterial({color: 0x0000ff});
// const meshZ = new THREE.Mesh(geometryZ, materialZ);




// function setVertexColors(geometry, color) {
//     const colors = [];
//     for (let i = 0; i < geometry.attributes.position.count; i++) {
//         colors.push(color.r, color.g, color.b);
//     }
//     geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
// }

// // Set vertex colors for each geometry
// setVertexColors(geometryX, new THREE.Color(0xff0000));
// setVertexColors(geometryY, new THREE.Color(0x0000ff));
// setVertexColors(geometryZ, new THREE.Color(0x00ff00));

// // Merge the geometries
// const mergedGeometry = BufferGeometryUtils.mergeBufferGeometries([geometryX, geometryY, geometryZ]);

// // Use a material that respects vertex colors
// const mergedMaterial = new THREE.MeshBasicMaterial({ vertexColors: true });

// const mergedMesh = new THREE.Mesh(mergedGeometry, mergedMaterial);
// scene.add(mergedMesh);

// Camera
const camera = new THREE.PerspectiveCamera(100, window.innerWidth / window.innerHeight, 0.6, 2000);
let globalCSVData = [];

function softmax(Tx, Ty, Tz) {
    let sum = (Tx) + (Ty) + (Tz);

    let wx = (Tx) / sum;
    let wy = (Ty) / sum;
    let wz = (Tz) / sum;

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
        Scene.frametoinistialstate();
    } else {
        let { wx, wy, wz } = softmax(report.Tx, report.Ty, report.Tz);
        //console.log(`XX: ${wx}, YY: ${wy}, ZZ: ${wz}`);
        
        // //Using Softmax
        // Scene.mergedMesh.position.x = report.Tx * wx / 100;
        // Scene.mergedMesh.position.z = report.Ty * wy / 100;
        // Scene.mergedMesh.position.y = report.Tz * wz / 100;

        //Observation 2

        Scene.mergedMesh.position.x = report.Tx/10
        Scene.mergedMesh.position.z = report.Ty/10
        Scene.mergedMesh.position.y = report.Tz/10

        console.log("TXX "+Scene.mergedMesh.position.x + " TYY " +Scene.mergedMesh.position.y +" TZZ " +Scene.mergedMesh.position.z ); // To log all positions at once
        //globalCSVData.push(['TXX', Scene.mergedMesh.position.x, 'TYY', Scene.mergedMesh.position.y, 'TZZ', Scene.mergedMesh.position.z]);
        DataManager.pushData(['TXX', Scene.mergedMesh.position.x, 'TYY', Scene.mergedMesh.position.y, 'TZZ', Scene.mergedMesh.position.z]);
    }

    if (report.Rx === 0 && report.Ry === 0 && report.Rz === 0) {
        Scene.frametoinistialstate();
    } else {
        let { wx, wy, wz } = softmax_r(report.Rx, report.Ry, report.Rz);
        //console.log(`XX: ${wx}, YY: ${wy}, ZZ: ${wz}`);
        
        // Scene.mergedMesh.rotation.x = report.Rx * wx / 450;
        // Scene.mergedMesh.rotation.z = report.Ry * wy / 450;
        // Scene.mergedMesh.rotation.y = report.Rz * wz / 450;
        console.log("RXX "+Scene.mergedMesh.rotation.x + " RYY " +Scene.mergedMesh.rotation.y +" RZZ " +Scene.mergedMesh.rotation.z ); // To log all positions at once
        //globalCSVData.push(['RXX', Scene.mergedMesh.rotation.x, 'RYY', Scene.mergedMesh.rotation.y, 'RZZ', Scene.mergedMesh.rotation.z]);
        DataManager.pushData(['RXX', Scene.mergedMesh.rotation.x, 'RYY', Scene.mergedMesh.rotation.y, 'RZZ', Scene.mergedMesh.rotation.z]);

      
    } // To log all rotations at once
    }
function arrayToCSV(data) {
        return data.map(row => row.join(',')).join('\n');
    }
function downloadCSV(csvContent, fileName = "data.csv") {
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const link = document.createElement("a");
    
    const url = URL.createObjectURL(blob);
    link.setAttribute("href", url);
    link.setAttribute("download", fileName);
    link.style.visibility = 'hidden';
    
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
}
    
    
// Set initial camera state when starting
Scene.setCameraToInitialState();


window.addEventListener('DOMContentLoaded', (event) => {
    const downloadButton = document.getElementById('downloadButton');
    downloadButton.addEventListener('click', downloadAccumulatedData);
});

function downloadAccumulatedData() {
    const csvContent = arrayToCSV(DataManager.getData());
    downloadCSV(csvContent, "output.csv");
}



// const axesHelper = new THREE.AxesHelper(10);  // 5 is the size; adjust as needed
// scene.add(axesHelper);


// Renderer
// const renderer = new THREE.WebGLRenderer({antialias: true});
// renderer.setClearColor("#233143"); // Set background colour
// renderer.setSize(window.innerWidth, window.innerHeight);
// document.body.appendChild(renderer.domElement); // Add renderer to HTML as a canvas element
// // Make Canvas Responsive
// window.addEventListener('resize', () => {
//     renderer.setSize(window.innerWidth, window.innerHeight); // Update size
//     camera.aspect = window.innerWidth / window.innerHeight; // Update aspect ratio
//     camera.updateProjectionMatrix(); // Apply changes
// })


// // Add ambient light
// const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
// scene.add(ambientLight);


// // Rendering
// const rendering = function() {
//     requestAnimationFrame(rendering);
//     renderer.setClearColor("#000000"); // Black background
//     renderer.render(scene, camera);
// }
// rendering();
