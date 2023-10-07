import {Color, Mesh} from 'three'
import * as THREE from 'three'
import {mergeGeometries} from 'three/addons/utils/BufferGeometryUtils.js'


class Axes extends Mesh {

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