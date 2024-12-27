# Candy Machine
interactive particle simulation (a technical and interaction study)

technology:  
TouchDesigner / compute shaders (GLSL) for physics / Leap Motion sensor

[<img src="https://i.vimeocdn.com/video/1846846394-c191eba5b33e39370178b5ffaf31f2f19558ef31f08b0d9aad48feb3bd6c3c54-d?mw=2880&mh=975&q=70" alt="demo video" width="65%">](https://vimeo.com/942039715/003609dd7c)


The Candy Machine is a work-in-progress series of small studies in technology and interaction, exploring the possibilities of shaders in TouchDesigner to model custom real-time physics for a vast amount of particles, combined with live interaction via hand gestures.
It is inspired by the pure joy of playing around with thousands of little gumball candies, made tangible by direct hand interaction. Once particle systems are populated with a huge number of entities, for me they become very captivating and mesmerizing to watch because of their granularity and the emergent patterns they form. Driven by an approximation and simplification of physical laws, they appear organic and artificial at the same time - constantly reshaping into abstract silhouettes before dissolving back into chaos.

This setup uses compute shaders to calculate the particle physics in parallel on the GPU. By using a spatial hash technique to organize the particles into a virtual grid of bins, this dramatically reduces the amount of collision checking required to ensure that each particle bounces off its neighbors. This combination greatly increases performance, allowing a larger number of particles to be simulated in realtime. To increase accuracy, position based dynamics are used, as well as implicit geometry (via signed distance functions) for flexible boundary checking.

The system can be interacted with through hand movements tracked by a Leap Motion sensor. The studies explore different approaches how to translate the hand tracking data into the simulation - from very trivial representations to rather abstract interpretations. 
