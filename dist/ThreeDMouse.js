import {DEVICE_SPECS, HID_FILTERS, IDS_TO_NAME} from "./deviceSpecs.js";

/**
 * Represents a 3D Mouse, which listens to WebHID events and emits custom events.
 */



export class ThreeDMouse {

    /**
     * Constructor for the ThreeDMouse class.
     * @param {HIDDevice} device - The WebHID device.
     * @param {Object} dataSpecs - The specifications for the device data.
     * @param {Object} options - Configuration options.
     * @param {boolean} options.emitRepeatedEvents - Flag to determine whether to emit repeated events.
     * @param {Function} options.filter - Filter function to process the raw input.
     */


    constructor(device, dataSpecs, {emitRepeatedEvents: emitRepeatedEvents=false, filter: filter}) {
        this.device = device;
        this.dataSpecs = dataSpecs
        this.filter = filter
        this.device.addEventListener("inputreport", this.handleInputReport.bind(this));
        navigator.hid.addEventListener("connect", this.handleConnectedDevice.bind(this));
        navigator.hid.addEventListener("disconnect", this.handleDisconnectedDevice.bind(this));
        // Leaving t as a sentinel value to make sure we don't fire a misleading empty event
        // when using `emitRepeatedEvents`.
        this._workingState =  {
            "t": -1,
            "x": 0.,
            "y": 0.,
            "z": 0.,
            "r": 0.,
            "p": 0.,
            "ya": 0.,
            "buttonsState": {},
            "buttonsValue": 0,
            "buttonsChanged": false,
            "controlChangeCount": 0,
        }
        this._emitRepeatedEventsInterval = null
        this._lastEmittedTime = Date.now();
        // The device is connected at this point, so start the repeat event timer
        // if the user requested it
        this.emitRepeatedEvents = emitRepeatedEvents
    }

    handleConnectedDevice(e) {
        // Rerun the setter to get the interval running
        this._emitRepeatedEvents = this._emitRepeatedEvents
        const event = new CustomEvent('3dmouseconnected', {
            bubbles: true,
            cancelable: true,
            detail: {
                mouse: this,
                hidEvent: e
            }
        });
        window.dispatchEvent(event)
    }

    /**
     * We redispatch the WebHID disconnected event so we can include a
     * reference to the owning ThreeDMouse instance.
     * @param e
     */
    handleDisconnectedDevice(e) {
        if (this._emitRepeatedEventsInterval) {
            window.clearInterval(this._emitRepeatedEventsInterval)
            this._emitRepeatedEventsInterval = null
        }
        const event = new CustomEvent('3dmousedisconnected', {
            bubbles: true,
            cancelable: true,
            detail: {
                mouse: this,
                hidEvent: e
            }
        });
        window.dispatchEvent(event)
    }

    /**
     * Getter and Setter for emitting repeated events.
     */

    set emitRepeatedEvents(value) {
        if (this._emitRepeatedEventsInterval === value) {
            // Idempotent
            return
        }
        this._emitRepeatedEvents = value
        if (this._emitRepeatedEvents) {
            this._emitRepeatedEventsInterval = window.setInterval(this._emitRepeatedEvent.bind(this), 1)
        } else {
            window.clearInterval(this._emitRepeatedEventsInterval)
            this._emitRepeatedEventsInterval = null
        }
    }
    /**
     * Emit a repeated event if the last emitted time exceeds a threshold.
     */

    _emitRepeatedEvent() {
        const currentTime = Date.now();
        if (currentTime - this._lastEmittedTime > 7) {
            this._workingState["t"] = currentTime
            window.dispatchEvent(this._makeEventFromState(this._workingState));
            this._lastEmittedTime = this._workingState["t"]
        }
    }

    /**
     * Static method to request a device. Opens the device if it's not already opened.
     * @param {Object} options - Options for the request.
     * @returns {ThreeDMouse} - A new instance of ThreeDMouse.
     */

    static async requestDevice(options) {
        const devices = await navigator.hid.requestDevice({ filters: HID_FILTERS });

        if (devices.length === 0) {
            console.warn("No devices found.");
            return;
        }

        this.device = devices[0];

        if (!this.device.opened) {
            await this.device.open();
            console.log("Opened device: " + this.device.productName);
            const driverName = IDS_TO_NAME[[this.device.vendorId, this.device.productId]];
            return new ThreeDMouse(this.device, DEVICE_SPECS[driverName], options)

        } else {
            console.log("Device is already open:", this.device.productName);
        }
      
    }

    /**
     * Handle WebHID events for a 3D mouse. Events typically come in at 7ms intervals
     * @param e
     */
    handleInputReport(e) {
        let data = e.data

        for (const [name, {channel: chan, byte: byte, scale: flip}] of Object.entries(this.dataSpecs.mappings)) {
            if (e.reportId === chan) {
            this._workingState[name] = flip * data.getInt16(byte, true) / this.dataSpecs.axisScale;
            if (name === "r" || name === "x") {
                this._workingState["controlChangeCount"] += 1;
            }
            }
        }

        let newButtonsValue = null
        for (let button_index = 0; button_index < this.dataSpecs.buttonMapping.length; button_index++) {
            const {name: name, channel: chan, byte: byte, bit: bit} = this.dataSpecs.buttonMapping[button_index];
            if (e.reportId === chan) {
                this._workingState["buttonsChanged"] = true;
                if (newButtonsValue === null) {
                    newButtonsValue = 0
                }
                // update the button vector
                const mask = 1 << bit;
                const state = data.getUint8(byte) & mask
                this._workingState["buttonsState"][name] = state !== 0;
                newButtonsValue |= state << (8 * byte)
            }
        }
            
        if (this._workingState["controlChangeCount"] <= 1 && !this._workingState["buttonsChanged"]) {
            return;
        }
        if (newButtonsValue !== null) {
            this._workingState["buttonsValue"] = newButtonsValue
        }
        this._workingState["t"] = Date.now();

        const outEvent = this._makeEventFromState(this._workingState)
        window.dispatchEvent(outEvent);
        this._lastEmittedTime = this._workingState["t"]
        this._workingState["controlChangeCount"] = 0;
        
    }
    /**
     * Construct a custom event from the current state.
     * @param {Object} state - The current working state.
     * @returns {Event} - A custom event with details.
     */

    _makeEventFromState(state) {
        const transIn = [state["x"], state["y"], state["z"]]
        const rotIn = [state["r"], state["p"], state["ya"]]
        let filtered = this.filter ? this.filter.process(transIn, rotIn): null;
        return new CustomEvent('3dmouseinput', {
            bubbles: true,
            cancelable: true,
            detail: {
                input: [transIn, rotIn],
                filteredInput: filtered,
                buttons: state["buttonsState"],
                buttonsValue: state["buttonsValue"],
                time: this._workingState["t"]
            }
        });
    }
}
