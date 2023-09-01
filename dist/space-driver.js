let device;
window.latestEvent = null;

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
export function configureCallback(callback) {
    dataCallback = callback
}


if (navigator.hid) {
    navigator.hid.addEventListener("connect", handleConnectedDevice);
    navigator.hid.addEventListener("disconnect", handleDisconnectedDevice);
} else {
    console.error("WebHID API is not supported in this browser.");
}

function handleConnectedDevice(e) { window.latestEvent = e;
   console.log("Device connected: " + e.device.productName);
}

function handleDisconnectedDevice(e) {window.latestEvent = e;
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


export function handleInputReport(e) {

   // console.log(e.reportId)
    // console.log(e)
    switch (e.reportId) {
        case 1:  // translation event
            response.Tx = e.data.getInt16(0, true);
            response.Ty = e.data.getInt16(2, true);
            response.Tz = e.data.getInt16(4, true);
            
            console.log(`Tx: ${response.Tx}, Ty: ${response.Ty}, Tz: ${response.Tz}`);
            document.getElementById("translateX").textContent = response.Tx;
            document.getElementById("translateY").textContent = response.Ty;
            document.getElementById("translateZ").textContent = response.Tz;
            freshResponse = false;
            break;

        case 2:  // rotation event
            response.Rx = e.data.getInt16(0, true);
            response.Ry = e.data.getInt16(2, true);
            response.Rz = e.data.getInt16(4, true);
            
            console.log(`Rx: ${response.Rx}, Ry: ${response.Ry}, Rz: ${response.Rz}`);
            document.getElementById("rotateX").textContent = response.Rx;
            document.getElementById("rotateY").textContent = response.Ry;
            document.getElementById("rotateZ").textContent = response.Rz;
            freshResponse = true;
            break;

        case 3:  // key press/release event
            // Handle key presses based on your requirements
            // No changes needed here for Tx, Ty, Tz, Rx, Ry, Rz
            break;
    }
    if (dataCallback !== null && freshResponse) {
        dataCallback(response)
    }

}
window.addEventListener('DOMContentLoaded', (event) => {
    const button = document.querySelector('button');
    button.addEventListener('click', selectDevice);
    navigator.hid.addEventListener("connect", handleConnectedDevice);
    navigator.hid.addEventListener("disconnect", handleDisconnectedDevice);
});