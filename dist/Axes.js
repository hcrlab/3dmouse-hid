import {Color, Mesh} from 'three'
import * as THREE from 'three'
import {mergeGeometries} from 'three/addons/utils/BufferGeometryUtils.js'


class Axes extends Mesh {

	constructor( {size: size = 1 , thickness: thickness= .5}) {
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
			new THREE.MeshPhysicalMaterial( {color: 0xff0000} ),
			new THREE.MeshPhysicalMaterial( {color: 0x00ff00} ),
			new THREE.MeshPhysicalMaterial( {color: 0x0000ff} )
			]

		super( merged, material );
		this.geometry.groups[0].materialIndex = 0
		this.geometry.groups[1].materialIndex = 1
		this.geometry.groups[2].materialIndex = 2
		this.type = 'Axes';

	}

	setColors( xAxisColor, yAxisColor, zAxisColor ) {
		return this
		const color = new Color();
		const array = this.geometry.attributes.color.array;

		color.set( xAxisColor );
		color.toArray( array, 0 );
		color.toArray( array, 3 );

		color.set( yAxisColor );
		color.toArray( array, 6 );
		color.toArray( array, 9 );

		color.set( zAxisColor );
		color.toArray( array, 12 );
		color.toArray( array, 15 );

		this.geometry.attributes.color.needsUpdate = true;

		return this;

	}


}


export { Axes };