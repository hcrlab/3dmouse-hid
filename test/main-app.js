import * as THREE from "https://unpkg.com/three@0.126.1/build/three.module.js"
import { BufferGeometryUtils } from "https://unpkg.com/three@0.126.1/examples/jsm/utils/BufferGeometryUtils.js";
// import { handleInputReport } from '../dist/space-driver.js';
import * as SpaceDriver from '../dist/space-driver.js';
import * as Scene from '../dist/scene.js';
import DataManager from '../dist/dataManager.js';
SpaceDriver.configureCallback(processInput)


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

function computePosition(value, threshold, growthRateAbove, baseBelow) {
    if (value < threshold) {
        return Math.pow(baseBelow, value);
    } else {
        return Math.pow(baseBelow, threshold) + growthRateAbove * (value - threshold);
    }
}
function processInput(report) {



    if (report.Tx === 0 && report.Ty === 0 && report.Tz === 0) {
        Scene.mergedMesh.position.x = 0
        Scene.mergedMesh.position.z = 0
        Scene.mergedMesh.position.y = 0
        console.log("TXX "+Scene.mergedMesh.position.x + " TYY " +Scene.mergedMesh.position.y +" TZZ " +Scene.mergedMesh.position.z ); // To log all positions at once
        DataManager.pushData(['TXX', Scene.mergedMesh.position.x, 'TYY', Scene.mergedMesh.position.y, 'TZZ', Scene.mergedMesh.position.z]);
        Scene.frametoinistialstate();
    } else {

        //Observation 3
        if(report.Tx <= 50 || report.Ty <=50 || report.Tz <= 50){
            Scene.frametoinistialstate();



        }

        //Using Softmax (Observation 1)
        let { wx, wy, wz } = softmax(report.Tx, report.Ty, report.Tz);
        //console.log(`XX: ${wx}, YY: ${wy}, ZZ: ${wz}`);
        
        // 
        // Scene.mergedMesh.position.x = report.Tx * wx / 100;
        // Scene.mergedMesh.position.z = report.Ty * wy / 100;
        // Scene.mergedMesh.position.y = report.Tz * wz / 100;

        //Observation 2
        // const threshold = 175;
        // const growthRateAboveThreshold = 10; // Linear growth rate for values above the threshold
        // const baseBelowThreshold = 1.02; // Exponential growth rate for values below the threshold
        
        // Scene.mergedMesh.position.x = computePosition(report.Tx, threshold, growthRateAboveThreshold, baseBelowThreshold);
        // Scene.mergedMesh.position.y = computePosition(report.Ty, threshold, growthRateAboveThreshold, baseBelowThreshold);
        // Scene.mergedMesh.position.z = computePosition(report.Tz, threshold, growthRateAboveThreshold, baseBelowThreshold);
        if(report.Tx >= 175 || report.Ty >= 175 || report.Tz >= 175){
            Scene.mergedMesh.position.x = report.Tx/1.5
            Scene.mergedMesh.position.z = report.Ty/1.5
            Scene.mergedMesh.position.y = report.Tz/1.5
            console.log("varad")
        }
        else{
            Scene.mergedMesh.position.x = report.Tx/2
            Scene.mergedMesh.position.z = report.Ty/2
            Scene.mergedMesh.position.y = report.Tz/2
        }

        
        console.log("TXX "+Scene.mergedMesh.position.x + " TYY " +Scene.mergedMesh.position.y +" TZZ " +Scene.mergedMesh.position.z ); // To log all positions at once
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


