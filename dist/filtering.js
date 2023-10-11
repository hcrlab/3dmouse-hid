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
            filtered = filtered.map((x, i) => (1 - this.smoothing) * x + vPrev[i] * this.smoothing)
        }
        return filtered
    }

}