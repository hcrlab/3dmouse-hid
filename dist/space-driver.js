import DataManager from './dataManager.js';

export class ThreeDMouse {
    constructor(device) {
        this.device = device;
        this.device.addEventListener("inputreport", this.handleInputReport.bind(this));
        navigator.hid.addEventListener("connect", this.handleConnectedDevice);
        navigator.hid.addEventListener("disconnect", this.handleDisconnectedDevice);
        this.dataCallback = null;
        this.dataCallback_1=null;
        this.freshResponse = false;
        this.response = {
            Tx: null,
            Ty: null,
            Tz: null,
            Rx: null,
            Ry: null,
            Rz: null
        };
        this.response_filter = {
            Tx_f: null,
            Ty_f: null,
            Tz_f: null,
            Rx_f: null,
            Ry_f: null,
            Rz_f: null
        };
        this.value_grip=null
    }
    configureCallback(callback) {
        this.dataCallback = callback
    }
    configure_grip(callback_1){
        this.dataCallback_1=callback_1

    }

    handleConnectedDevice(e) {
        console.log(`Device ${e.device.productName} is connected.`);
    }
    handleDisconnectedDevice(e) {
        console.log(`Device ${e.device.productName} is disconnected.`);
    }
    static async requestDevice() {
        try {
            const devices = await navigator.hid.requestDevice({ filters: [{ vendorId: 0x046d }] });

            if (devices.length === 0) {
                console.warn("No devices found.");
                return;
            }

            this.device = devices[0];

            if (!this.device.opened) {
                await this.device.open();
                console.log("Opened device: " + this.device.productName);
                return new ThreeDMouse(this.device)

            } else {
                console.log("Device is already open:", this.device.productName);
            }
        } catch (error) {
            console.error("Error:", error);
        }
    }

    handleInputReport(e) {

        let response_filter = {
            Tx_f: null,
            Ty_f: null,
            Tz_f: null,
            Rx_f: null,
            Ry_f: null,
            Rz_f: null
        }
        switch (e.reportId) {
            case 1: // Translation event

                // Get translation values from the data
                this.response.Tx = e.data.getInt16(0, true) / 350;
                this.response.Ty = e.data.getInt16(2, true) / 350;
                this.response.Tz = e.data.getInt16(4, true) / 350;

                // Display translation values in the console and on the webpage
                console.log(`Tx: ${this.response.Tx}, Ty: ${this.response.Ty}, Tz: ${this.response.Tz}`);

                // Push raw translation data to DataManager
                DataManager.pushData(['Tx', this.response.Tx, 'Ty', this.response.Ty, 'Tz', this.response.Tz]);

                  // Push raw translation data to DataManager
            DataManager.pushData(['Tx', this.response.Tx, 'Ty', this.response.Ty, 'Tz', this.response.Tz]);

            // If translation values are all zero, push zeros to DataManager
            if (this.response.Tx === 0 && this.response.Ty === 0 && this.response.Tz === 0) {
                DataManager.pushData(['TXX', 0, 'TYY', 0, 'TZZ', 0]);
            }


            /**
             * Declare the values (before filtering, we assign the raw values from the mouse to the 
             * variables which will eventually have filtered values).
             */

            response_filter.Tx_f = this.response.Tx;
            response_filter.Ty_f = this.response.Ty;
            response_filter.Tz_f = this.response.Tz;

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
            
            this.freshResponse=false;
            this.response_filter.Tx_f = response_filter.Tx_f;
            this.response_filter.Ty_f = response_filter.Ty_f;
            this.response_filter.Tz_f = response_filter.Tz_f;
            break;

            
        case 2: // Rotation event

            // Get rotation values from the data
            this.response.Rx = e.data.getInt16(0, true) / 350;
            this.response.Ry = e.data.getInt16(2, true) / 350;
            this.response.Rz = e.data.getInt16(4, true) / 350;
        
            // Display rotation values in the console and on the webpage
            console.log(`Rx: ${this.response.Rx}, Ry: ${this.response.Ry}, Rz: ${this.response.Rz}`);
        
            // Push raw rotation data to DataManager
            DataManager.pushData(['Rx', this.response.Rx, 'Ry', this.response.Ry, 'Rz', this.response.Rz]);
        
            // Declare the values for filtering
            response_filter.Rx_f = this.response.Rx;
            response_filter.Ry_f = this.response.Ry;
            response_filter.Rz_f = this.response.Rz;
        
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
        
            this.freshResponse = true;

            this.response_filter.Rx_f = response_filter.Rx_f;
            this.response_filter.Ry_f = response_filter.Ry_f; 
            this.response_filter.Rz_f = response_filter.Rz_f;
            break;

        case 3:  // key press/release event
            // Handle key presses based on your requirements
            // No changes needed here for Tx, Ty, Tz, Rx, Ry, Rz
            const value = e.data.getUint8(0);
            
            /*
                For my SpaceNavigator, a device having two (2) keys only:
                value is a 2-bit bitmask, allowing 4 key-states:
                value = 0: no keys pressed
                value = 1: left key pressed
                value = 2: right key pressed
                value = 3: both keys pressed
                */
            console.log(value)
            this.value_grip=value
            console.log("Left key " + ((value & 1) ? "pressed," : "released,") + "   Right key " + ((value & 2) ? "pressed, " : "released;"));
            break;
			
			default:		// just in case a device exhibits unexpected capabilities  8-)
				console.log(e.device.productName + ": Received UNEXPECTED input report " + e.reportId);
				console.log(new Uint8Array(e.data.buffer));
           
            break;
    }
    if (this.dataCallback !== null && this.freshResponse) {
        this.dataCallback(this.response_filter)
    }
    if (this.dataCallback_1 !== null) {
        this.dataCallback_1(this.value_grip)
    }
   
    
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

