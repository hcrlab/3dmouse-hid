import * as THREE from 'three'

/**
 * Rotate a given twist using a rotation matrix.
 * 
 * @param {Array} twist - A two-element array: [linear, angular], where both linear and angular are vectors.
 * @param {THREE.Matrix3} rotMatrix - A 3x3 matrix used for the rotation.
 * @returns {Array} - The rotated twist as [rotatedLinear, rotatedAngular].
 */


export function rotateTwist(twist, rotMatrix) {
    let [linear, angular] = [ensureVector3(twist[0]), ensureVector3(twist[1])]
    return [linear.applyMatrix3(rotMatrix), angular.applyMatrix3(rotMatrix)]
}

/**
 * Ensures the given input is a THREE.Vector3 instance. If not, attempts to convert it.
 * 
 * @param {THREE.Vector3 | Array} input - The input to be converted/ensured.
 * @returns {THREE.Vector3} - The input as a THREE.Vector3.
 * @throws {Error} Throws an error if the input cannot be converted.
 */

export function ensureVector3(input) {
    if (input instanceof THREE.Vector3) {
        return input; // If it's already a Vector3, no conversion needed.
    } else if (Array.isArray(input) && input.length === 3) {
        return new THREE.Vector3().fromArray(input); // Convert from array.
    } else {
        throw new Error('Input is not a valid THREE.Vector3 or array of length 3.');
    }
}

/**
 * Integrate a given twist in a stepwise manner over a specified time.
 * 
 * @param {Array} twist - A two-element array: [linear, angular], representing the linear and angular components of the twist.
 * @param {number} time - The total time over which the twist is to be integrated.
 * @param {number} steps - The number of discrete steps for the integration.
 * @returns {Array} - An array of THREE.Vector3 points that result from the stepwise integration.
 */

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
