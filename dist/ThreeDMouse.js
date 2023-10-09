import { HID_FILTERS, IDS_TO_NAME, DEVICE_SPECS } from "./device_specs.js";


function makeCubicDeadbandFilter(weight, deadband) {
    const d3 = Math.pow(deadband, 3)
    const iw = 1 - weight
    const scaleFactor = weight * d3 + iw * deadband
    return (x) => {
        if (Math.abs(x) < deadband || x === 0) {
            return 0;
        }
       return ((weight * Math.pow(x, 3) + iw * x) - ((Math.abs(x) / x) * scaleFactor)) / (1 - scaleFactor)
    }
}

export class SmoothingDeadbandFilter {
    constructor({
                    deadband: deadband=0.1,
                    cubicDeadbandWeight: cubicDeadbandWeight = 0.3,
                    smoothing: smoothingWeight=0.5,
                    softmaxWeight: softmaxWeight=0.5,
                    translationMultiplier: translationMultiplier = 1.0,
                    rotationMultiplier: rotationMultiplier = 1.0}) {
        this.smoothing = smoothingWeight
        this._deadband = deadband
        this._cubicDeadbandWeight = cubicDeadbandWeight
        this.softmaxWeight = softmaxWeight
        this._deadbandFilter = makeCubicDeadbandFilter(cubicDeadbandWeight, deadband)
        this.translationMultiplier = translationMultiplier
        this.rotationMultiplier = rotationMultiplier
        this._previousOutput = [[0, 0, 0], [0, 0, 0]]
    }

    set deadband(value) {
        this._deadband = value
        this._deadbandFilter = makeCubicDeadbandFilter(this._cubicDeadbandWeight, this._deadband)
    }

    set cubicDeadbandWeight(value) {
        this._cubicDeadbandWeight = value
        this._deadbandFilter = makeCubicDeadbandFilter(this._cubicDeadbandWeight, this._deadband)
    }

    process(transIn, rotIn) {
        let transProcessed = this._processComponent(transIn, this._previousOutput[0], this.translationMultiplier)
        let rotProcessed = this._processComponent(rotIn, this._previousOutput[1], this.rotationMultiplier)
        this._previousOutput = [transProcessed, rotProcessed]
        return [transProcessed, rotProcessed]
    }

    _processComponent(v, vPrev, scaleFactor=1.0) {
        let filtered = v.map(this._deadbandFilter)
        // L1 norm
        let filteredNorm = filtered.reduce((a, b) => a + Math.abs(b), 0)
        filteredNorm = Math.min(filteredNorm, 1.0)
        if (filteredNorm === 0.0) {
            // Result is just decay of previous input
            filtered = vPrev.map(x => x * this.smoothing)
        } else {
            const filteredExp = filtered.map(x => Math.exp(Math.abs(x) / this.softmaxWeight))
            const transExpNorm = filteredExp.reduce((a, b) => a + b, 0)
            let softmax = filteredExp.map(x => x / transExpNorm)
            filtered = filtered.map((x, i) => x * softmax[i])
            const softmaxedNorm = filtered.reduce((a, b) => a + Math.abs(b), 0)
            filtered = filtered.map(x => x * (filteredNorm / softmaxedNorm) * scaleFactor)
            filtered = filtered.map((x, i) => (1- this.smoothing) * x +  vPrev[i] * this.smoothing)
        }
        return filtered
    }

}

export class ThreeDMouse {
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
            "buttonStates": {},
            "buttonsValue": 0,
            "buttonsChanged": false,
            "controlChangeCount": 0,
        }
        this._emitRepeatedEventsInterval = null
        this._lastEmittedTime = Date.now() / 1000;
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

    _emitRepeatedEvent() {
        const currentTime = Date.now() / 1000;
        if (currentTime - this._lastEmittedTime > .007) {
            this._workingState["t"] = currentTime
            window.dispatchEvent(this._makeEventFromState(this._workingState));
            this._lastEmittedTime = this._workingState["t"]
        }
    }
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
        
        for (let button_index = 0; button_index < this.dataSpecs.buttonMapping.length; button_index++) {
            const {name: name, channel: chan, byte: byte, bit: bit} = this.dataSpecs.buttonMapping[button_index];
            if (e.reportId === chan) {
                this._workingState["buttonsChanged"] = true;
                // update the button vector
                const mask = 1 << bit;
                this._workingState["buttonStates"][name] = (data.getUint8(byte) & mask) !== 0 ? 1 : 0;
                this._workingState["buttonsValue"] |= mask
            }
        }
            
        if (this._workingState["controlChangeCount"] <= 1 && !this._workingState["buttonsChanged"]) {
            return;
        }
        this._workingState["t"] = Date.now() / 1000;

        const outEvent = this._makeEventFromState(this._workingState)
        window.dispatchEvent(outEvent);
        this._lastEmittedTime = this._workingState["t"]
        this._workingState["controlChangeCount"] = 0;
        this._workingState["buttonsValue"] = 0
        
    }

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
                buttons: state["buttons"],
                time: this._workingState["t"]
            }
        });
    }
}
