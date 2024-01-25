# Candy Machine
interactive particle simulation (a technical and interaction study)

![simulation still](http://cv.simonstimberg.de/assets/artworks/CandyMachine07.jpg "inside the candy machine")


The project is based solely on the pure joy of experiencing thousands of little gumball candy particles, bubbling around in 3D space. Once particle systems are set with a huge amount of entities, they can become very captivating and mesmerising to look at, because of their granularity and emergent patterns they form. Seeking to mimic nature, but in a digital setting, they appear organic and artificial at the same time - constantly reshaping into abstract silhouettes before dissolving again into chaos.	

This setup uses compute shaders to calculate the particle physics in parallel on the GPU. By using a spatial hash technique to organise the particles into a virtual grid of bins, this reduces dramatically the amount of collision checks that are necessary for each particle to bounce off their neighbors. This combination allows for a great increase in performance and therefore enables to simulate a larger amount of particles in realtime.									

The system can be interacted with through hand movements, tracked by a Leap Motion sensor. By tilting the left hand, the perspective of the cube can be controlled, while the right hand’s position is used to repulse the particles.
	
