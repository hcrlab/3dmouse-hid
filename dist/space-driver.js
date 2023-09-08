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
    let sum = Math.abs(Tx) + Math.abs(Ty) + Math.abs(Tz);

    

    return {
        wx: Math.abs(Tx)/sum,
        wy: Math.abs(Ty)/sum,     
        wz: Math.abs(Tz)/sum
    };
}

function softmax_r(Rx, Ry, Rz) {
    let sum = Math.abs(Rx) + Math.abs(Ry) + Math.abs(Rz);
    console.log("RX"+ Rx)
    console.log("SUM" + sum)


    return {
        WX: Math.abs(Rx) / sum,
        WY: Math.abs(Ry) / sum,
        WZ: Math.abs(Rz) / sum
    };
        
    
}

export function handleInputReport(e) {

   // console.log(e.reportId)
    // console.log(e)
    switch (e.reportId) {
        case 1: // Translation event

            // Get translation values from the data
            response.Tx = e.data.getInt16(0, true) / 350;
            response.Ty = e.data.getInt16(2, true) / 350;
            response.Tz = e.data.getInt16(4, true) / 350;

            // Display translation values in the console and on the webpage
            console.log(`Tx: ${response.Tx}, Ty: ${response.Ty}, Tz: ${response.Tz}`);
            document.getElementById("translateX").textContent = response.Tx;
            document.getElementById("translateY").textContent = response.Ty;
            document.getElementById("translateZ").textContent = response.Tz;

            // Push raw translation data to DataManager
            DataManager.pushData(['Tx', response.Tx, 'Ty', response.Ty, 'Tz', response.Tz]);

            // If translation values are all zero, push zeros to DataManager
            if (response.Tx === 0 && response.Ty === 0 && response.Tz === 0) {
                DataManager.pushData(['TXX', 0, 'TYY', 0, 'TZZ', 0]);
            }

            /**
             * Declare the values (before filtering, we assign the raw values from the mouse to the 
             * variables which will eventually have filtered values).
             */

            response_filter.Tx_f = response.Tx;
            response_filter.Ty_f = response.Ty;
            response_filter.Tz_f = response.Tz;

            // Condition 1: Filter values within the range [-0.2, 0.2]
            response_filter.Tx_f = (-0.2 <= response_filter.Tx_f && response_filter.Tx_f <= 0.2) ? 0.0 : response_filter.Tx_f;
            response_filter.Ty_f = (-0.2 <= response_filter.Ty_f && response_filter.Ty_f <= 0.2) ? 0.0 : response_filter.Ty_f;
            response_filter.Tz_f = (-0.2 <= response_filter.Tz_f && response_filter.Tz_f <= 0.2) ? 0.0 : response_filter.Tz_f;

            // Condition 2: Divide values within the range [-0.5, 0.5] by 1.5
            response_filter.Tx_f = (-0.5 <= response_filter.Tx_f && response_filter.Tx_f <= 0.5) ? response_filter.Tx_f / 1.5 : response_filter.Tx_f;
            response_filter.Ty_f = (-0.5 <= response_filter.Ty_f && response_filter.Ty_f <= 0.5) ? response_filter.Ty_f / 1.5 : response_filter.Ty_f;
            response_filter.Tz_f = (-0.5 <= response_filter.Tz_f && response_filter.Tz_f <= 0.5) ? response_filter.Tz_f / 1.5 : response_filter.Tz_f;

            // Condition 3: Apply Softmax function for non-zero values
            if (response_filter.Tx_f != 0 && response_filter.Ty_f != 0 && response_filter.Tz_f != 0){
                let { wx, wy, wz } = softmax(response_filter.Tx_f, response_filter.Ty_f, response_filter.Tz_f);
                response_filter.Tx_f = response_filter.Tx_f * wx;
                response_filter.Ty_f = response_filter.Ty_f * wy;
                response_filter.Tz_f = response_filter.Tz_f * wz;
            }
            
            // Push filtered translation data to DataManager
            console.log("TXX"+ response_filter.Tx_f, "TYY" +response_filter.Ty_f, "TZZ"+ response_filter.Tz_f)
            //DataManager.pushData(['TXX', response_filter.Tx_f, 'TYY', response_filter.Ty_f, 'TZZ', response_filter.Tz_f]);
            
            freshResponse=false;

            
        case 2: // Rotation event

            // Get rotation values from the data
            response.Rx = e.data.getInt16(0, true) / 350;
            response.Ry = e.data.getInt16(2, true) / 350;
            response.Rz = e.data.getInt16(4, true) / 350;
        
            // Display rotation values in the console and on the webpage
            console.log(`Rx: ${response.Rx}, Ry: ${response.Ry}, Rz: ${response.Rz}`);
            document.getElementById("rotateX").textContent = response.Rx;
            document.getElementById("rotateY").textContent = response.Ry;
            document.getElementById("rotateZ").textContent = response.Rz;
        
            // Push raw rotation data to DataManager
            DataManager.pushData(['Rx', response.Rx, 'Ry', response.Ry, 'Rz', response.Rz]);
        
            // Declare the values for filtering
            response_filter.Rx_f = response.Rx;
            response_filter.Ry_f = response.Ry;
            response_filter.Rz_f = response.Rz;
        
            // Condition_1: Filter values within the range [-0.2, 0.2]
            response_filter.Rx_f = (-0.2 <= response_filter.Rx_f && response_filter.Rx_f <= 0.2) ? 0.0 : response_filter.Rx_f;
            response_filter.Ry_f = (-0.2 <= response_filter.Ry_f && response_filter.Ry_f <= 0.2) ? 0.0 : response_filter.Ry_f;
            response_filter.Rz_f = (-0.2 <= response_filter.Rz_f && response_filter.Rz_f <= 0.2) ? 0.0 : response_filter.Rz_f;
        
            // Condition_2: Divide values within the range [-0.5, 0.5] by 1.5
            response_filter.Rx_f = (0.5 >= response_filter.Rx_f && response_filter.Rx_f >= -0.5) ? response_filter.Rx_f / 1.5 : response_filter.Rx_f;
            response_filter.Ry_f = (0.5 >= response_filter.Ry_f && response_filter.Ry_f >= -0.5) ? response_filter.Ry_f / 1.5 : response_filter.Ry_f;
            response_filter.Rz_f = (0.5 >= response_filter.Rz_f && response_filter.Rz_f >= -0.5) ? response_filter.Rz_f / 1.5 : response_filter.Rz_f;
        
            // Condition 3: Apply Softmax function for non-zero values
            if (response_filter.Rx_f != 0 && response_filter.Ry_f != 0 && response_filter.Rz_f != 0) {
                // Apply Softmax function and update filtered values
                let { WX, WY, WZ } = softmax_r(response_filter.Rx_f, response_filter.Ry_f, response_filter.Rz_f);
                response_filter.Rx_f = response_filter.Rx_f * WX;
                response_filter.Ry_f = response_filter.Ry_f * WY;
                response_filter.Rz_f = response_filter.Rz_f * WZ;
            }
        
            // Display filtered values and push them to DataManager
            console.log("RXX" + response_filter.Rx_f , "RYY"+ response_filter.Ry_f, "RZZ"+response_filter.Rz_f);
            //DataManager.pushData(['RXX', response_filter.Rx_f, 'RYY', response_filter.Ry_f, 'RZZ', response_filter.Rz_f]);
        
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