import {Color, Mesh} from 'three'
import * as THREE from 'three'
import {mergeGeometries} from 'three/addons/utils/BufferGeometryUtils.js'


/**
 * Represents a 3D set of axes (X, Y, Z) using cylinders in a 3D space.
 * Extends the Mesh class from the three.js library.
 * Each axis is color-coded: Red for X, Green for Y, and Blue for Z by default.
 */

class Axes extends Mesh {


/**
     * Constructs the Axes mesh object.
     * 
     * @param {Object} options - Configuration options for the axes.
     * @param {number} [options.size=1] - Specifies the length of each axis. 
     * @param {number} [options.thickness=0.05] - Defines the thickness or diameter of each cylindrical axis.
     * @param {number} [options.opacity=1] - Sets the opacity of the axes; 1 being fully opaque and less than 1 being translucent.
     */


	constructor( {size: size = 1 , thickness: thickness= .05, opacity: opacity = 1}) {

		const xGeometry = new THREE.CylinderGeometry(thickness, thickness, size, 32 );
		xGeometry.rotateZ(Math.PI / 2)
		xGeometry.translate(size / 2, 0, 0)
		const yGeometry = new THREE.CylinderGeometry(thickness,thickness, size,32);
		yGeometry.rotateY(Math.PI / 2)
		yGeometry.translate(0, size / 2, 0)
		const zGeometry = new THREE.CylinderGeometry(thickness,thickness, size,20,32);
		zGeometry.rotateX(Math.PI / 2)
		zGeometry.translate(0, 0, size / 2)
		const merged = mergeGeometries([xGeometry, yGeometry, zGeometry], true)

		let material = [
			new THREE.MeshPhysicalMaterial( {color: 0xff0000, opacity: opacity, transparent: opacity < 1.0} ),
			new THREE.MeshPhysicalMaterial( {color: 0x00ff00, opacity: opacity, transparent: opacity < 1.0} ),
			new THREE.MeshPhysicalMaterial( {color: 0x0000ff, opacity: opacity, transparent: opacity < 1.0} )
			]

		super( merged, material );
		this._opacity = opacity
		this.geometry.groups[0].materialIndex = 0
		this.geometry.groups[1].materialIndex = 1
		this.geometry.groups[2].materialIndex = 2
		this.type = 'Axes';
	}
	
	/**
     * Sets the colors of the X, Y, and Z axes. 
     * Useful if the default colors (Red for X, Green for Y, Blue for Z) need to be changed.
     * 
     * @param {string|number} xAxisColor - Color for the X-axis, can be a hexadecimal, string, or RGB format.
     * @param {string|number} yAxisColor - Color for the Y-axis, can be a hexadecimal, string, or RGB format.
     * @param {string|number} zAxisColor - Color for the Z-axis, can be a hexadecimal, string, or RGB format.
     * @returns {Axes} Returns the updated Axes object, allowing for method chaining.
     */

	setColors( xAxisColor, yAxisColor, zAxisColor ) {
		this.material[0].color = new Color(xAxisColor)
		this.material[1].color = new Color(yAxisColor)
		this.material[2].color = new Color(zAxisColor)
		this.material[0].opacity = this._opacity
		this.material[1].opacity = this._opacity
		this.material[2].opacity = this._opacity
		return this;
	}
}


export { Axes };