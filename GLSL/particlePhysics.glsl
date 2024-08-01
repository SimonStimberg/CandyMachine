// Example Compute Shader
layout (local_size_x = 8, local_size_y = 8) in;


// uniform float exampleUniform;
// uniform ivec3 uSimRes;	// we can take the resolution also from the TD parameters
uniform vec3 uBoundingBox;
uniform vec2 uMousePos;
uniform float uRepRadius;

uniform ivec3 uBinRes;
uniform int uBinDepth;

// uniform vec3 uAttrPosA;
// uniform vec3 uAttrPosB;
uniform vec3 uAttrPosArray[4];

uniform vec4 uLeapHandA;
// uniform vec4 uLeapHandB;

ivec2 simRes = ivec2(uTDOutputInfo.res.zw);
ivec2 binTex = ivec2(uTD2DInfos[4].res.zw); 


const float friction = 0.96;
const float step = 1.0 / 90.0;



float map(float value, float min1, float max1, float min2, float max2) {
  return min2 + (value - min1) * (max2 - min2) / (max1 - min1);
}

float length2( vec3 c) {
    return dot( c, c );
}

vec3 limitVec(vec3 inVec, float maxFrc) { 
    if (length2(inVec) > maxFrc*maxFrc) {
	    inVec = normalize(inVec) * maxFrc;
    }
    return inVec;
}


vec3 frcRepulsion(vec3 p1, vec3 p2, float radius, float strength) {
	vec3 diff = p1 - p2;
	float dist = dot(diff, diff);
	if (dist != 0.0 && dist < radius*radius)
	{
		dist = sqrt(dist);
		diff /= dist;
		// if (strength > 0.0) {
			diff *= map(dist, 0.0, radius, 1.0, 0.1);
		// } else {
		// 	diff *= map(dist, 0.0, radius, 0.8, 0.1);
		// 	// dist = clamp(dist, 0.2, radius);
		// 	// if(dist > 0.2) {
		// 		// diff *= map(dist, 0.2, radius, 1.0, 0.1);
		// 	// }
		// }
		return diff * strength;
	}
	return vec3(0.0);
}



vec3 searchVoxel(vec3 myPos, int myID, ivec3 vox, float repRadius) {

	vec3 newFrc = vec3(0.0);

	// if the voxel is out of bounds, return 0.0 (this happens when the particle is at the edge of the bounding box, so the scanning of the neighboring voxels would go out of bounds)
	if ( any(lessThan(vox, ivec3(0))) || any(greaterThan(vox, uBinRes-ivec3(1))) ) {	
		return newFrc;
	}


	int binID = vox.x + vox.y*uBinRes.x + vox.z*uBinRes.x*uBinRes.y;	// get the binID from the voxel (fold 3D into 1D)	
	int slotID = binID*(uBinDepth); // (bin ID * binDepth (= number of Slots per bin)) -> first slot is the start of the bin	

	// iterate over all slots of the bin
	for (int i = 0; i < uBinDepth; i++) {

		ivec2 slotCoord = ivec2(slotID%binTex.x, slotID/binTex.x);
		
		int otherID = int(texelFetch(sTD2DInputs[4], slotCoord, 0).r);		// get the ID of the particle from the bin texture a the corresponding slot


		// if the ID is -1 it means that there is no particle in this slot, so we can break the loop
		if (otherID == -1) {
			break;
		}

		// for debugging 
		// newFrc.z = float(otherID);
		// newFrc.xy = slotCoord * 1.0;
		// newFrc.x = float(myID);


		if (myID != otherID) {	// avoid check agains self

			ivec2 otherCoord = ivec2(otherID%simRes.x, otherID/simRes.x);
			vec3 otherPos = texelFetch(sTD2DInputs[0], otherCoord, 0).xyz;	// get the position of the other particle from the position texture
			
			newFrc += frcRepulsion(myPos, otherPos, repRadius, 20.0);		// do the collision check and add the force to the total force
		}

		slotID += 1;	// increment the slotID to iterate over all slots of the bin	
	}

	return newFrc;
}



void main()
{
	

	vec4 data = texelFetch(sTD2DInputs[0], ivec2(gl_GlobalInvocationID.xy), 0);		// get the data of the particle from the position texture
	vec3 pos = data.xyz;
	float life = data.a;

	vec3 vel = texelFetch(sTD2DInputs[1], ivec2(gl_GlobalInvocationID.xy), 0).xyz;
	vec3 noise = texelFetch(sTD2DInputs[2], ivec2(gl_GlobalInvocationID.xy), 0).xyz;
	float repRadius = uRepRadius * texelFetch(sTD2DInputs[3], ivec2(gl_GlobalInvocationID.xy), 0).r;
	// float repRadius = uRepRadius;

	ivec2 coord = ivec2(gl_GlobalInvocationID.xy);    
    int idx = coord.x + coord.y*simRes.x;	// get the particle ID

	// vec4 debug = vec4(0.0);



	ivec3 vox = ivec3(floor( (vec3(.5)+ min(pos, 0.4999))*uBinRes));		// get the Voxel the particle is in	-	clamp the pos below 0.5 to avoid running out of bounds


	vec3 frc = vec3(0.0);
	frc += vec3(noise * 0.7);


	// iterate over a kernal of 3x3x3 voxels - to check not only the voxel the particle is in, but also the neighboring voxels
	for (int z = -1; z <= 1; z++) {
		for (int y = -1; y <= 1; y++) {
			for (int x = -1; x <= 1; x++) {

				ivec3 d = ivec3(x, y, z);
				frc += searchVoxel(pos, idx, vox + d.xyz, repRadius);

			}
		}
	}


	// repell from mouse position
	// frc += frcRepulsion(pos, vec3(uMousePos, 0.0), 0.5, 100.0);

	vec3 futurePos = pos + vel * step;

	// repell from leap hands (palm position)
	if (uLeapHandA.a > -10.0) {
		frc += frcRepulsion(futurePos, uLeapHandA.xyz, 0.35, 200.0);
	}
	// if (uLeapHandB.w > 0.0) {
	// 	frc += frcRepulsion(pos, uLeapHandB.xyz, 0.5, 100.0);
	// }


	// int attrID = idx % 4;

	// // attract to the attractor positions
	// frc += frcRepulsion(pos, uAttrPosArray[attrID], 1., -2.);
	// frc += frcRepulsion(pos, uAttrPosArray[attrID], 1., -2.);
	// frc += frcRepulsion(pos, uAttrPosArray[attrID], 1., -2.);
	// frc += frcRepulsion(pos, uAttrPosArray[attrID], 1., -2.);
 

	frc += frcRepulsion(pos, uAttrPosArray[0], 0.75, -2.);
	frc += frcRepulsion(pos, uAttrPosArray[1], 0.75, -2.);
	frc += frcRepulsion(pos, uAttrPosArray[2], 0.75, -2.);
	frc += frcRepulsion(pos, uAttrPosArray[3], 0.75, -2.);


	// vec3 gravity = vec3(0.0, -1.7, 0.0);

	// frc += gravity;




	frc = limitVec(frc, 6.0);	// limit the force to avoid crazy jumps

	// add the force to the velocity and update the position
	vel *= friction;
	vel += frc * step;
	pos += vel * step;


	// do the bound checking
	if (pos.x < -uBoundingBox.x * 0.5 || pos.x > uBoundingBox.x * 0.5)
	{
		vel.x *= -1.0 * friction;
		pos.x = clamp(pos.x, -uBoundingBox.x * 0.5, uBoundingBox.x * 0.5);
	}
	if (pos.y < -uBoundingBox.y * 0.5 || pos.y > uBoundingBox.y * 0.5)
	{
		vel.y *= -1.0 * friction;
		pos.y = clamp(pos.y, -uBoundingBox.y * 0.5, uBoundingBox.y * 0.5);
	}
	if (pos.z < -uBoundingBox.z * 0.5 || pos.z > uBoundingBox.z * 0.5)
	{
		vel.z *= -1.0 * friction;
		pos.z = clamp(pos.z, -uBoundingBox.z * 0.5, uBoundingBox.z * 0.5);
	}



	imageStore(mTDComputeOutputs[0], ivec2(gl_GlobalInvocationID.xy), TDOutputSwizzle(vec4(pos, life)));	// write the new position
	imageStore(mTDComputeOutputs[1], ivec2(gl_GlobalInvocationID.xy), TDOutputSwizzle(vec4(vel, 1.0 )));					// write the new velocity 
	// imageStore(mTDComputeOutputs[2], ivec2(gl_GlobalInvocationID.xy), TDOutputSwizzle(debug));					// for debug purposes
}
