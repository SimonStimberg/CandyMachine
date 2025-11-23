// compute shader for binning particles into a 3D voxel grid

layout (local_size_x = 1, local_size_y = 1) in;
// layout (local_size_x = 8, local_size_y = 8) in;

ivec2 inRes = ivec2(uTD2DInfos[0].res.zw);			// the resolution of the input texture keeping the position of the particles (here the values x + y are normalized: 1/width and 1/height - while z and w are the actual width and height)
ivec2 uBinResShape = ivec2(uTDOutputInfo.res.zw);	// same for the output texture, which is are the bins

// uniform float exampleUniform;
uniform atomic_uint uBinCounters[40000];	// the bin counter must be >= the number of bins! if the bins exceed this number, particles will be omitted. Unfotunately, this number cannot be set dynamically, so it must be set to a high enough number to accomodate the maximum number of bins that might be created.

uniform ivec3 uBinRes;
uniform int uBinDepth;
uniform vec3 uWorldSize;

// binning code based on code by David Brown https://github.com/DBraun
void main()
{

	// can be used to offset the voxel grid in world space
	vec3 uVoxelOrigin = vec3(0, 0, 0);
	vec3 uVoxelWorld  = vec3(1, 1, 1);

	
	int id = int(gl_GlobalInvocationID.x);	// get the ID within the dispatch (as this is a 1D dispatch (1024,1,1), we only need the x component) -> the dispatch size is the num of Particles! so this is the ID of the particle
	
	ivec2 coord = ivec2(id%inRes.x, id/inRes.x);	// the 2D coordinate of the particle in the input texture (which holds the position of the particles)
	
	vec4 inData = texelFetch(sTD2DInputs[0], coord, 0);		// get the data of that Particle (the data is the position + the alpha value)
	
	if (inData.a == 0.) {
		// ignore the point because its alpha is zero
		return;
	}
	
	vec3 worldSpacePos = inData.xyz;	// only the position of the particle

	// worldSpacePos = min(worldSpacePos, vec3(0.4999));	// refrains from the edge case bug that if the position is exactly 0.5 it gets sorted in into the next line
	// vec3 edge = uWorldSize * 0.5 - vec3(0.0001);	// the world size minus a small value to prevent the edge case bug
	// worldSpacePos = min(worldSpacePos, edge);
	// worldSpacePos = min(worldSpacePos, vec3(0.3999));
	
	ivec3 vox = ivec3(floor( ( vec3(.5) + (worldSpacePos-uVoxelOrigin)/uWorldSize)*uBinRes));
	
	// ivec3 vox = ivec3(floor( (vec3(.5)+worldSpacePos)*uBinRes));



	// when the particle is out of bounds, we clamp it to the edge of the bounds
	if ( any(lessThan(vox, ivec3(0))) ) {
		vox = ivec3(0); 
	} else if ( any(greaterThan(vox, uBinRes-ivec3(1))) ) {
		vox = uBinRes-ivec3(1);
	}


		
	int binID = int( vox.z * uBinRes.x*uBinRes.y + vox.y * uBinRes.x + vox.x );		// caluate the bin ID (which is the index of the 3D voxel foled into an 1D array)

	uint numInBin = atomicCounterIncrement(uBinCounters[binID]);	// increments the counter at the position of the bin + returns the ORIGINAL value of the counter

	
	// if the bin should happen to be full, we don't write the particle to the bin - just stop here
	if (numInBin > uBinDepth-1) {
		return;
	} 

	int slotID = binID * uBinDepth;	// get the index of the specific bin in the texture (multiplied by the depth of the bin = the max number of particles a bin can have)	
	slotID += int(numInBin);		// add the counter incremention to get a free index in the bin (if the bin already holds particles)

	ivec2 binCoord = ivec2(slotID%uBinResShape.x, slotID/uBinResShape.x);	 	// get the 2D coordinate of that index
	

	imageStore(mTDComputeOutputs[0], binCoord, vec4(id));	// finally, write the particle ID into the bin at the free index

}


