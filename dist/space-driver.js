import DataManager from './dataManager.js';

let device;


let dataCallback = null;
let freshResponse = false;
let response = {
    Tx: null,
    Ty: null,
    Tz: null,
    Rx: null,
    Ry: null,
    Rz: null
};
let response_filter = {
    Tx_f: null,
    Ty_f: null,
    Tz_f: null,
    Rx_f: null,
    Ry_f: null,
    Rz_f: null
};
export function configureCallback(callback) {
    dataCallback = callback
}


if (navigator.hid) {
    navigator.hid.addEventListener("connect", handleConnectedDevice);
    navigator.hid.addEventListener("disconnect", handleDisconnectedDevice);
} else {
    console.error("WebHID API is not supported in this browser.");
}

function handleConnectedDevice(e) { 
   console.log("Device connected: " + e.device.productName);
}

function handleDisconnectedDevice(e) {
   console.log("Device disconnected: " + e.device.productName);
   console.dir(e);
   

}

export async function selectDevice() {
    try {
        const devices = await navigator.hid.requestDevice({ filters: [{ vendorId: 0x046d }] });

        if (devices.length === 0) {
            console.warn("No devices found.");
            return;
        }

        const device = devices[0];

        if (!device.opened) {
            await device.open();
            console.log("Opened device: " + device.productName);
           
            device.addEventListener("inputreport", handleInputReport);
            
        } else {
            console.log("Device is already open:", device.productName);
        }
    } catch (error) {
        console.error("Error:", error);
    }
}

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

export function handleInputReport(e) {

   // console.log(e.reportId)
    // console.log(e)
    switch (e.reportId) {
        case 1:  // translation event
            response.Tx = e.data.getInt16(0, true)/350;
            response.Ty = e.data.getInt16(2, true)/350;
            response.Tz = e.data.getInt16(4, true)/350;
            
            console.log(`Tx: ${response.Tx}, Ty: ${response.Ty}, Tz: ${response.Tz}`);
            document.getElementById("translateX").textContent = response.Tx;
            document.getElementById("translateY").textContent = response.Ty;
            document.getElementById("translateZ").textContent = response.Tz;
            DataManager.pushData(['Tx', response.Tx, 'Ty', response.Ty, 'Tz', response.Tz]);
            
            
            //filtered code
            //Observation (1) Keeping dead zone for values like 0.2 and -0.2 are considered as 0
            // Check if any of the Tx, Ty, or Tz values are outside the [-0.2, 0.2] range
        if (response.Tx <= 0.2 & response.Ty <= 0.2  & response.Tz <= 0.2 ) {
            response_filter.Tx_f = 0;
            response_filter.Ty_f = 0;
            response_filter.Tz_f = 0;
            //console.log("Obs 1" + response_filter.Tx_f);
        } 
        response_filter.Tx_f= (-0.2 <= response.Tz  || response.Tz <= 0.2) ? 0.0 : response.Tx_
        // What would the above line look like in Python:
        // response_filtered_x = 0.0 if -0.2 <= response.Tz or response.Tz <= 0.2 else response_x
        respones_filter.Ty_f=response.Ty
        response_filter.Tz_f=response.Tz
        if(response.Tx <= 0.2) {
            response_filter.Tx_f = 0;
            console.log("varad_1")
        }
        else if (response.Ty <= 0.2){
            response_filter.Ty_f = 0;
            console.log("varad_2")
        }
        else if(-0.2 <= response.Tz  || response.Tz <= 0.2){
            response_filter.Tz_f = 0;
            console.log("varad_3")

        }
        else {
            response_filter.Tx_f=response.Tx
            console.log("varad4")
            response_filter.Ty_f=response.Ty
            response_filter.Tz_f=response.Tz
        }
        // Check if any of the Tx, Ty, or Tz values are outside the [-0.5, 0.5] range
        if (response_filter.Tx_f >= 0.5 || response.Tx <= -0.5 || response_filter.Ty_f >= 0.5 || response.Ty <= -0.5 || response_filter.Tz_f >= 0.5 || response.Tz <= -0.5) {
            response_filter.Tx_f = response_filter.Tx_f/1
            response_filter.Ty_f = response_filter.Tx_f/1
            response_filter.Tz_f = response_filter.Tx_f/1
            console.log("Obs 1 " + response_filter.Tx_f)
            console.log("Obs 2 " + response_filter.Ty_f)
            console.log("Obs 3 " + response_filter.Tz_f)

            

            let {wx, wy, wz} = softmax(response_filter.Tx_f, response_filter.Ty_f, response_filter.Tz_f);
            console.log("XX" + wx)
            response_filter.Tx_f = response_filter.Tx_f * wx;
            response_filter.Ty_f = response_filter.Ty_f * wy;
            response_filter.Tz_f = response_filter.Tz_f * wz;
            console.log("Filtered X  " + response_filter.Tx_f,"Filtered Y  " + response_filter.Ty_f, "FIltered Z" + response_filter.Tz_f);
        } 
        // Check if any of the Tx, Ty, or Tz values are outside the [-0.2, 0.2] range and apply softmax
        else if (response.Tx <= 0.5 || response.Tx >= -0.5 || response.Ty <= 0.5 || response.Ty >= -0.5 || response.Tz <= 0.5 || response.Tz >= -0.2) {
            response_filter.Tx_f = response.Tx / 2;
            response_filter.Ty_f = response.Ty / 2;
            response_filter.Tz_f = response.Tz / 2;


            let {wx, wy, wz} = softmax(response.Tx, response.Ty, response.Tz);
            response_filter.Tx_f = response_filter.Tx_f * wx;
            response_filter.Ty_f = response_filter.Ty_f * wy;
            response_filter.Tz_f = response_filter.Tz_f * wz;
            //console.log("Obs 2 and 3  " + response_filter.Tx_f);
        } 
        
    
            freshResponse = false;
            break;

        case 2:  // rotation event
            response.Rx = e.data.getInt16(0, true)/350;
            response.Ry = e.data.getInt16(2, true)/350;
            response.Rz = e.data.getInt16(4, true)/350;
            
            console.log(`Rx: ${response.Rx}, Ry: ${response.Ry}, Rz: ${response.Rz}`);
            document.getElementById("rotateX").textContent = response.Rx;
            document.getElementById("rotateY").textContent = response.Ry;
            document.getElementById("rotateZ").textContent = response.Rz;
            DataManager.pushData(['Rx', response.Rx, 'Ry', response.Ry, 'Rz', response.Rz]);
            freshResponse = true;
            break;

        case 3:  // key press/release event
            // Handle key presses based on your requirements
            // No changes needed here for Tx, Ty, Tz, Rx, Ry, Rz
            break;
    }
    if (dataCallback !== null && freshResponse) {
        dataCallback(response_filter)
    }




}
window.addEventListener('DOMContentLoaded', (event) => {
    const button = document.querySelector('button');
    button.addEventListener('click', selectDevice);   
});