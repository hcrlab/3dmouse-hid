import * as THREE from "https://unpkg.com/three@0.126.1/build/three.module.js"
import { BufferGeometryUtils } from "https://unpkg.com/three@0.126.1/examples/jsm/utils/BufferGeometryUtils.js";
// import { handleInputReport } from '../dist/space-driver.js';
import * as SpaceDriver from '../dist/space-driver.js';
import * as Scene from '../dist/scene.js';
import DataManager from '../dist/dataManager.js';
SpaceDriver.configureCallback(processInput)


function processInput(report) {
    if (report.Tx_f === 0 && report.Ty_f === 0 && report.Tz_f === 0) {
        Scene.frametoinistialstate();
    }

    Scene.mergedMesh.position.x = -report.Tx_f
    Scene.mergedMesh.position.z = -report.Ty_f
    Scene.mergedMesh.position.y = -report.Tz_f

    Scene.mergedMesh.rotation.x = -report.Rx_f 
    Scene.mergedMesh.rotation.z = -report.Ry_f 
    Scene.mergedMesh.rotation.y = -report.Rz_f


    // console.log("TXX "+Scene.mergedMesh.position.x + " TYY " +Scene.mergedMesh.position.y +" TZZ " +Scene.mergedMesh.position.z ); // To log all positions at once
    // console.log("RXX "+Scene.mergedMesh.rotation.x + " RYY " +Scene.mergedMesh.rotation.y +" RZZ " +Scene.mergedMesh.rotation.z )
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


