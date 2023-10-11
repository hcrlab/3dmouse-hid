
/**
 * Creates a cubic deadband filter for input values.
 *
 * @param {number} weight - Weight assigned to cubic deadband.
 * @param {number} deadband - Deadband value for the filter.
 * @returns {function} - A filter function that will apply a cubic deadband based on the given weight and deadband.
 */



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

/**
 * Class representing a smoothing filter with deadband for processing input values.
 * This can be used for inputs like joystick movements where minor inputs are to be ignored 
 * (deadband) and remaining inputs are to be smoothed out.
 */

export class SmoothingDeadbandFilter {


     /**
     * Constructs the filter.
     * 
     * @param {Object} options - Configuration options for the filter.
     * @param {number} [options.deadband=0.1] - Deadband value.
     * @param {number} [options.cubicDeadbandWeight=0.3] - Weight assigned to cubic deadband.
     * @param {number} [options.smoothingWeight=0.5] - Weight for the smoothing component.
     * @param {number} [options.softmaxTemperature=0.5] - Temperature for softmax calculation.
     * @param {number} [options.translationMultiplier=1.0] - Multiplier for translation components.
     * @param {number} [options.rotationMultiplier=1.0] - Multiplier for rotation components.
     */


    constructor({
                    deadband: deadband = 0.1,
                    cubicDeadbandWeight: cubicDeadbandWeight = 0.3,
                    smoothing: smoothingWeight = 0.5,
                    softmaxTemperature: softmaxTemperature = 0.5,
                    translationMultiplier: translationMultiplier = 1.0,
                    rotationMultiplier: rotationMultiplier = 1.0
                }) {
        this.smoothing = smoothingWeight
        this._deadband = deadband
        this._cubicDeadbandWeight = cubicDeadbandWeight
        this.softmaxTemperature = softmaxTemperature
        this._deadbandFilter = makeCubicDeadbandFilter(cubicDeadbandWeight, deadband)
        this.translationMultiplier = translationMultiplier
        this.rotationMultiplier = rotationMultiplier
        this._previousOutput = [[0, 0, 0], [0, 0, 0]]
    }

    /**
     * Setter for deadband. Updates the deadband filter upon change.
     */

    set deadband(value) {
        this._deadband = value
        this._deadbandFilter = makeCubicDeadbandFilter(this._cubicDeadbandWeight, this._deadband)
    }

     /**
     * Setter for cubic deadband weight. Updates the deadband filter upon change.
     */

    set cubicDeadbandWeight(value) {
        this._cubicDeadbandWeight = value
        this._deadbandFilter = makeCubicDeadbandFilter(this._cubicDeadbandWeight, this._deadband)
    }

    /**
     * Process the input values (translation and rotation) using the filter.
     *
     * @param {number[]} transIn - Input values for translation.
     * @param {number[]} rotIn - Input values for rotation.
     * @returns {number[][]} - Filtered values for translation and rotation.
     */

    process(transIn, rotIn) {
        let transProcessed = this._processComponent(transIn, this._previousOutput[0], this.translationMultiplier)
        let rotProcessed = this._processComponent(rotIn, this._previousOutput[1], this.rotationMultiplier)
        this._previousOutput = [transProcessed, rotProcessed]
        return [transProcessed, rotProcessed]
    }

      /**
     * Process a specific component (translation or rotation) of the input values using the filter.
     *
     * @private
     * @param {number[]} v - Input values for the component.
     * @param {number[]} vPrev - Previous values for the component.
     * @param {number} [scaleFactor=1.0] - Scaling factor for the component.
     * @returns {number[]} - Filtered values for the component.
     */


    _processComponent(v, vPrev, scaleFactor = 1.0) {
        let filtered = v.map(this._deadbandFilter)
        // L1 norm
        let filteredNorm = filtered.reduce((a, b) => a + Math.abs(b), 0)
        filteredNorm = Math.min(filteredNorm, 1.0)
        if (filteredNorm === 0.0) {
            // Result is just decay of previous input
            filtered = vPrev.map(x => x * this.smoothing)
        } else {
            const filteredExp = filtered.map(x => Math.exp(Math.abs(x) / this.softmaxTemperature))
            const transExpNorm = filteredExp.reduce((a, b) => a + b, 0)
            let softmax = filteredExp.map(x => x / transExpNorm)
            filtered = filtered.map((x, i) => x * softmax[i])
            const softmaxedNorm = filtered.reduce((a, b) => a + Math.abs(b), 0)
            filtered = filtered.map(x => x * (filteredNorm / softmaxedNorm) * scaleFactor)
            // Combine the filtered value with the previous value for smoothing
            filtered = filtered.map((x, i) => (1 - this.smoothing) * x + vPrev[i] * this.smoothing)
        }
        return filtered
    }

}