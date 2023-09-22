import * as THREE from "https://unpkg.com/three@0.126.1/build/three.module.js"
import { BufferGeometryUtils } from "https://unpkg.com/three@0.126.1/examples/jsm/utils/BufferGeometryUtils.js";



// Scene
const scene = new THREE.Scene();
export function setCameraToInitialState() {
    camera.position.set(19, 5, 2);
    camera.lookAt(0, 0, 0);
}


export function frametoinistialstate(){
    mergedMesh.position.x=0;
    mergedMesh.position.y=0;
    mergedMesh.position.z=0;
    mergedMesh.rotation.x =0;
    mergedMesh.rotation.y =0;
    mergedMesh.rotation.z =0;

}


//Red X-axis
const curveX = new THREE.LineCurve3(
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(10, 0, 0)
);
const geometryX = new THREE.TubeBufferGeometry(curveX, 64, 0.1, 8, false);
const materialX = new THREE.MeshLambertMaterial({color: 0xff0000});
const meshX = new THREE.Mesh(geometryX, materialX);

//Red X-axis (Static)
const curve_X = new THREE.LineCurve3(
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(10, 0, 0)
);
const geometry_X = new THREE.TubeBufferGeometry(curve_X, 64, 0.1, 8, false);
const material_X = new THREE.MeshLambertMaterial({color: 0xff0000});
const mesh_X = new THREE.Mesh(geometry_X, material_X);


// Green Y-axis
const curveY = new THREE.LineCurve3(
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(0, 10, 0)
);
const geometryY = new THREE.TubeBufferGeometry(curveY, 64, 0.1, 8, false);
const materialY = new THREE.MeshLambertMaterial({color: 0x00ff00});
const meshY = new THREE.Mesh(geometryY, materialY);

// Green Y-axis  (static)
const curve_Y = new THREE.LineCurve3(
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(0, 10, 0)
);
const geometry_Y = new THREE.TubeBufferGeometry(curve_Y, 64, 0.1, 8, false);
const material_Y = new THREE.MeshLambertMaterial({color: 0x00ff00});
const mesh_Y = new THREE.Mesh(geometry_Y, material_Y);

 // Blue Z-axis
const curveZ = new THREE.LineCurve3(
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(0, 0, -10)
);
const geometryZ = new THREE.TubeBufferGeometry(curveZ, 64, 0.1, 8, false);
const materialZ = new THREE.MeshLambertMaterial({color: 0x0000ff});
const meshZ = new THREE.Mesh(geometryZ, materialZ);

 // Blue Z-axis
 const curve_Z = new THREE.LineCurve3(
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(0, 0, -10)
);
const geometry_Z = new THREE.TubeBufferGeometry(curve_Z, 64, 0.1, 8, false);
const material_Z = new THREE.MeshLambertMaterial({color: 0x0000ff});
const mesh_Z = new THREE.Mesh(geometry_Z, material_Z);




export function setVertexColors(geometry, color) {
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

export const mergedMesh = new THREE.Mesh(mergedGeometry, mergedMaterial);
scene.add(mergedMesh);


// Merge the static geometries
// const staticMergedGeometry = BufferGeometryUtils.mergeBufferGeometries([geometry_X, geometry_Y, geometry_Z]);
// const staticMergedMesh = new THREE.Mesh(staticMergedGeometry, mergedMaterial);
// scene.add(staticMergedMesh);

scene.add(mesh_X);
scene.add(mesh_Y);
scene.add(mesh_Z);





// Camera
const camera = new THREE.PerspectiveCamera(100, window.innerWidth / window.innerHeight, 0.6, 100);
const desiredWidth = 2500; // Set the width you desire
const desiredHeight = 500; // Set the height you desire


// Renderer
const renderer = new THREE.WebGLRenderer({antialias: true})
renderer.setClearColor("#233143"); // Set background colour
renderer.setSize(desiredWidth, desiredHeight);
document.body.appendChild(renderer.domElement); // Add renderer to HTML as a canvas element
// Make Canvas Responsive
window.addEventListener('resize', () => {
    // Get the current window size
    const newWidth = window.innerWidth;
    const newHeight = window.innerHeight;

    // Calculate the aspect ratio based on desired width and height
    const aspectRatio = desiredWidth / desiredHeight;

    // Calculate the new size while maintaining the aspect ratio
    let finalWidth, finalHeight;
    if (newWidth / newHeight > aspectRatio) {
        finalWidth = newHeight * aspectRatio;
        finalHeight = newHeight;
    } else {
        finalWidth = newWidth;
        finalHeight = newWidth / aspectRatio;
    }

    // Update the renderer size and camera aspect ratio
    renderer.setSize(finalWidth, finalHeight);
    camera.aspect = aspectRatio;
    camera.updateProjectionMatrix();
});



// Add ambient light
const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
scene.add(ambientLight);


// Rendering
const rendering = function() {
    requestAnimationFrame(rendering);
    renderer.setClearColor("#000000"); // Black background;
    renderer.render(scene, camera);
}
rendering();