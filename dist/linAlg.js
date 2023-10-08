import * as THREE from 'three'

export function rotateTwist(twist, rotMatrix) {
    let [linear, angular] = [ensureVector3(twist[0]), ensureVector3(twist[1])]
    return [linear.applyMatrix3(rotMatrix), angular.applyMatrix3(rotMatrix)]
}

export function ensureVector3(input) {
    if (input instanceof THREE.Vector3) {
        return input; // If it's already a Vector3, no conversion needed.
    } else if (Array.isArray(input) && input.length === 3) {
        return new THREE.Vector3().fromArray(input); // Convert from array.
    } else {
        throw new Error('Input is not a valid THREE.Vector3 or array of length 3.');
    }
}

export function integrateTwistStepwise(twist, time, steps) {
    let [linear, angular] = [ensureVector3(twist[0]), ensureVector3(twist[1])]
    const dt = time / steps
    let points = [new THREE.Vector3(0,0,0)]
    let stepRotation = angular.clone().multiplyScalar(dt)
    let linearStep = linear.clone().multiplyScalar(dt)
    for (let i = 1; i < steps; i++) {
        // New point is the previous rotated then translated
        const newPoint = points[i - 1].clone()
        newPoint.applyEuler(new THREE.Euler(...stepRotation)).add(linearStep)
        points.push(newPoint)
    }
    return points
}
