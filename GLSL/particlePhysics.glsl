// Compute Shader for Particle Simulation with SDF Constraints and binning based collision detection


layout (local_size_x = 8, local_size_y = 8) in;

// uniforms aka parameters from the outer world
uniform float uRadius;
uniform float uConstraintRadius;
uniform int uSubSteps;

uniform ivec3 uBinRes;
uniform int uBinDepth;

uniform vec3 uMousePos;
uniform float uSdfDiv;
uniform vec3 uWorldSize;
uniform mat4 uTransMat;
uniform vec2 uFriction;

bool bFriction = (uFriction.x > 0.5);
float friction = uFriction.y;

const int leapPts = 21;
uniform vec2 uLeapStatus;
layout(binding = 0) uniform samplerBuffer uLeapPos;
layout(binding = 1) uniform samplerBuffer uLeapRad;
layout(binding = 2) uniform samplerBuffer uTest;



ivec2 simRes = ivec2(uTDOutputInfo.res.zw);
int offset = simRes.x / 2;
ivec2 binTex = ivec2(uTD2DInfos[2].res.zw); 


// global variables

int frameRate = 640 * uSubSteps;
float stepSize = 1.0 / frameRate;
const int subSteps = 1;

vec3  pos;
vec3  posPrev;
vec3  acceleration = vec3(0.0);
float radius;




float map(float value, float min1, float max1, float min2, float max2) {
  return min2 + (value - min1) * (max2 - min2) / (max1 - min1);
}

// float dot2( in vec2 v ) { return dot(v,v); }


// signed distance fields + related functions 
// by https://iquilezles.org/articles/


float sdSphere( vec3 p, float s )
{
  return length(p)-s;
}

float sdBox( vec3 p, vec3 b )
{
  vec3 q = abs(p) - b;
  return length(max(q,0.0)) + min(max(q.x,max(q.y,q.z)),0.0);
}


float opUnion( float d1, float d2 )
{
    return min(d1,d2);
}

float opSmoothUnion( float d1, float d2, float k )
{
    float h = clamp( 0.5 + 0.5*(d2-d1)/k, 0.0, 1.0 );
    return mix( d2, d1, h ) - k*h*(1.0-h);
}


float sdScene( vec3 p)
{
	float sdScn = 0.0;

	float r = uConstraintRadius - radius;
	sdScn = sdSphere(p, r);
	// sdScn = sdBox(p, vec3(r));

	return sdScn;
}

// for 3D 
vec3 calcNormal( in vec3 p ) // for function f(p)
{
    // const float eps = 0.0001; // or some other value
    // const vec2 h = vec2(eps,0);
    // return normalize( vec3(sdScene(p+h.xyy) - sdScene(p-h.xyy),
    //                        sdScene(p+h.yxy) - sdScene(p-h.yxy),
    //                        sdScene(p+h.yyx) - sdScene(p-h.yyx) ) );


	// optimized Normal Calculation by Inigo Quilez
	// https://www.shadertoy.com/view/lt3BW2
	const float ep = 0.0001;
    vec2 e = vec2(1.0,-1.0)*0.5773;
    return normalize( e.xyy*sdScene( p + e.xyy*ep ) + 
					  e.yyx*sdScene( p + e.yyx*ep ) + 
					  e.yxy*sdScene( p + e.yxy*ep ) + 
					  e.xxx*sdScene( p + e.xxx*ep ) );
}



// physics functions

vec3 getVelocity()
{
	return (pos - posPrev) / stepSize;
	// const dvec3 pPos = dvec3(pos);
	// const dvec3 pPosPrev = dvec3(posPrev);
	// const double dt = double(stepSize);
	// return (pPos - pPosPrev) / dt;
}

void setVelocity(vec3 v)
{
	posPrev = pos - (v * stepSize);
}

void applyFriction()
{
	// float friction = 0.9990;
	vec3 tempVelocity = getVelocity();
	tempVelocity *= friction;
	setVelocity(tempVelocity);
}

void limitVelocity(float l)
{
	vec3 vel = getVelocity();
	float mag = dot(vel, vel);
	if (mag > l*l)
	{
		mag = sqrt(mag);
		vel /= mag;
		vel *= l;
		// tempVelocity = normalize(tempVelocity) * l;
		setVelocity(vel);
	}
}


vec3 applyGravity()
{
	float force = 100.0;
	float mass = 1.0 + texelFetch(sTD2DInputs[1], ivec2(gl_GlobalInvocationID.xy), 0).r * 2.;
	force *= mass;

	vec3 direction = vec3(0.0, -1.0, 0.0);
	vec3 transformed_direction = vec3(uTransMat * vec4(direction, 0.0));	// take account rotation of the world
	
	return transformed_direction * force;
}


// repulsion/attraction force from/to a point
vec3 frcRepulsion(vec3 point, float repRadius, float strength) {

	const vec3 diff = pos - point;
	const float dist2 = dot(diff, diff);

	if (dist2 != 0.0 && dist2 < repRadius*repRadius)
	{
		const float dist = sqrt(dist2);
		const vec3 n = diff / dist;
		// if (strength > 0.0) {
		const vec3 frc = n * map(dist, 0.0, repRadius, 0.0, 1.0)  * strength;
		// } else {
		// 	diff *= map(dist, 0.0, radius, 0.8, 0.1);
		// 	// dist = clamp(dist, 0.2, radius);
		// 	// if(dist > 0.2) {
		// 		// diff *= map(dist, 0.2, radius, 1.0, 0.1);
		// 	// }
		// }
		if (bFriction) applyFriction();

		return frc;
	}
	return vec3(0.0);
}



void solveCollision(vec3 otherPos, float otherRadius)
{
	const float    response_coef = 0.1f;		// 0.05f
	
	const vec3  v = pos - otherPos;
	const float dist2    = dot(v, v);
	const float min_dist = radius + otherRadius;
	// Check overlapping
	if (dist2 < min_dist * min_dist) {
		const float dist  = sqrt(dist2);
		const vec3  n     = v / dist;
		const float mass_ratio_1 = radius / (radius + otherRadius);
		const float mass_ratio_2 = otherRadius / (radius + otherRadius);
		const float delta        = 0.5f * response_coef * (dist - min_dist);
		// Update positions
		pos -= n * (mass_ratio_2 * delta);
		// object_1.position -= n * (mass_ratio_2 * delta);
		// object_2.position += n * (mass_ratio_1 * delta);
		if (bFriction) applyFriction();
	}

}



// search a given voxel for neighboring particles and solve collision with them
void searchVoxel(int myID, ivec3 vox) {



	// if the voxel is out of bounds, return 0.0 (this happens when the particle is at the edge of the bounding box, so the scanning of the neighboring voxels would go out of bounds)
	if ( any(lessThan(vox, ivec3(0))) || any(greaterThan(vox, uBinRes-ivec3(1))) ) {	
		return;
	}


	int binID = vox.x + vox.y*uBinRes.x + vox.z*uBinRes.x*uBinRes.y;	// get the binID from the voxel (fold 3D into 1D)	
	int slotID = binID*(uBinDepth); // (bin ID * binDepth (= number of Slots per bin)) -> first slot is the start of the bin	

	// iterate over all slots of the bin
	for (int i = 0; i < uBinDepth; i++) {

		ivec2 slotCoord = ivec2(slotID%binTex.x, slotID/binTex.x);
		
		int otherID = int(texelFetch(sTD2DInputs[2], slotCoord, 0).r);		// get the ID of the particle from the bin texture a the corresponding slot


		// if the ID is -1 it means that there is no particle in this slot, so we can break the loop
		if (otherID == -1) {
			break;
		}
		else if (myID != otherID) {	// avoid check agains self

			const ivec2 otherCoord = ivec2(otherID%simRes.x, otherID/simRes.x);
			vec3  otherPos 	  = texelFetch(sTD2DInputs[0], otherCoord, 0).xyz;
			float otherRadius = texelFetch(sTD2DInputs[1], otherCoord, 0).r * uRadius;

			solveCollision(otherPos, otherRadius);
		}

		slotID += 1;	// increment the slotID to iterate over all slots of the bin	
	}

}


// check collision in all adjecent voxels
void checkCollisions()
{
	ivec2 coord = ivec2(gl_GlobalInvocationID.xy);    
    int idx = coord.x + coord.y*simRes.x;	// get the particle ID

	// ivec3 vox = ivec3(floor( (vec3(.5)+ min(vec3(pos, 0.0), 0.4999))*uBinRes));		// get the Voxel the particle is in	-	clamp the pos below 0.5 to avoid running out of bounds
	
	// vec3 edge = uWorldSize * 0.5 - vec3(0.0001);	// the world size minus a small value to prevent the edge case bug
	// ivec3 vox = ivec3(floor( (uWorldSize/2 + min(vec3(pos, 0.0), edge))*uBinRes));
	// ivec3 vox = ivec3(floor( (vec3(.4) + min(vec3(pos, 0.0), 0.3999))*uBinRes));
	ivec3 vox = ivec3(floor( ( vec3(.5) + pos/uWorldSize)*uBinRes));


	// iterate over a kernal of 3x3x3 voxels - to check not only the voxel the particle is in, but also the neighboring voxels
	for (int z = -1; z <= 1; z++) {
		for (int y = -1; y <= 1; y++) {
			for (int x = -1; x <= 1; x++) {

				ivec3 d = ivec3(x, y, z);
				// frc += searchVoxel(pos, idx, vox + d.xyz, repRadius);
				searchVoxel(idx, vox + d.xyz);

			}
		}
	}

}


// apply SDF based constraint
void applyConstraint(float dt)
{
	float sd = sdScene(pos);

	if (sd > 0.0) {
		vec3 n = calcNormal(pos);
		pos = pos -  n * sd;
		if (bFriction) applyFriction();
	}
}


// update position based on Verlet integration
void update(float dt)
{
	// Compute how much we moved
	const vec3 displacement = pos - posPrev;
	// Update position
	posPrev = pos;
	pos      = pos + displacement + acceleration * (dt * dt);

	// limitVelocity(10.0);
	
}



void main()
{
	simRes.x -= offset;

	ivec2 id = ivec2(gl_GlobalInvocationID.xy);
	// split texture into two halves for current and previous position
	// by using one merged texture, multi-pass computation is possible
	if (id.x < simRes.x) {	

		ivec2 idPrev = id;
		idPrev.x += offset;

		// get particle data
		pos 	= texelFetch(sTD2DInputs[0], ivec2(id), 0).xyz;
		posPrev = texelFetch(sTD2DInputs[0], ivec2(idPrev), 0).xyz;
		radius  = texelFetch(sTD2DInputs[1], ivec2(id), 0).r * uRadius;


		// add gravity
		acceleration += applyGravity();		

		// LEAP MOTION interaction
		if(uLeapStatus.x > 0.0) {

			// solve collision for each hand joint -> as spheres
			for (int i = 0; i < leapPts ; i++) { 
				vec3 segPos = texelFetch(uLeapPos, i).xyz;
				float segRad = texelFetch(uLeapRad, i).r;

				solveCollision(segPos, segRad);
			}

			// if the hand is grabbing, apply attraction forces to all hand joints
			if(uLeapStatus.y > 0.7) {
				for (int i = 0; i < leapPts; i++) { 
					vec3 segPos = texelFetch(uLeapPos, i).xyz;
					acceleration += frcRepulsion(segPos, 0.24, -130.0); 
				}
			}			
		}

		// check collision against other particles (using spatial binning -> onlay against particles in neighboring voxels)
		checkCollisions();

		// apply SDF based constraint
		applyConstraint(stepSize);

		// finally update position
		update(stepSize);


		// write back updated particle data to the output textures

		// first texture: current position
		imageStore(mTDComputeOutputs[0], ivec2(id), TDOutputSwizzle(vec4(pos, 1.0)));
		// second texture: previous position (for position based velocity calculation - aka Verlet integration)
		imageStore(mTDComputeOutputs[0], ivec2(idPrev), TDOutputSwizzle(vec4(posPrev, 1.0)));


		// FOR DEBUGGING
		// float testVal = texelFetch(uLeapRad, int(gl_GlobalInvocationID.x % 21)).r;
		// imageStore(mTDComputeOutputs[1], ivec2(gl_GlobalInvocationID.xy), TDOutputSwizzle(vec4(testVal, 1.0, 1.0, 1.0)));

	}
}
