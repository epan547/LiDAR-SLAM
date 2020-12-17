---
title: About
layout: landing
description: 'About this project'
image: assets/images/pic07.jpg
nav-menu: true
---

<!-- Main -->
<div id="main">

<!-- One -->
<section id="2D LiDAR Based SLAM">
	<div class="inner">
		<header class="major">
			<h2>Sed amet aliquam</h2>
		</header>
		<p>The goal of our project was to implement a basic SLAM alorithm using a physical Neato from Olin. SLAM is a class of algorithms for simaltaneously mapping an area and locating the robot within that map. We used the Neato's built-in LiDAR and wheel encoders to sense the environment, and a method called Graph Optimization to map it.</p>
	</div>
</section>

<!-- Two -->
<section id="Graph Optimization" class="spotlights">
	<section>
		<a href="generic.html" class="image">
			<img src="{% link assets/images/Driffft.png %}" alt="" data-position="center center" />
		</a>
		<div class="content">
			<div class="inner">
				<header class="major">
					<h3>Orci maecenas</h3>
				</header>
				<p>To use graph optimization for SLAM, we recorded the locations of the robot and its LiDAR measurements in the odometry reference frame as nodes on a graph. We recorded the distance between these points, as measured by the sensors, as edges between those nodes. So long as all the measurements agree, there is no optimization to be done, and we can plot the data directly. However, none of our sensors are perfect, so we are likely to get a graph that resembles the one to the left. </p>
			</div>
		</div>
	</section>
	<section>
		<a href="generic.html" class="image">
			<img src="{% link assets/images/pic09.jpg %}" alt="" data-position="top center" />
		</a>
		<div class="content">
			<div class="inner">
				<header class="major">
					<h3>Loop Closure</h3>
				</header>
				<p>Once we have multiple measurements of the same landmark or location, we will have conflicting information because the sensors will never entirely agree. Graph Optimization optimizes the entire graph to reconcile the mismatched readings with the minimum possible change. Finding a landmark that you have re-scanned is called Loop Closure, and it is what enables graph optimization to be more than a simple plot of sensor data. </p>
				<ul class="actions">
					<li><a href="generic.html" class="button">Learn more</a></li>
				</ul>
			</div>
		</div>
	</section>
	<section>
		<a href="generic.html" class="image">
			<img src="{% link assets/images/projected!stable!scan!.png %}" alt="" data-position="25% 25%" />
		</a>
		<div class="content">
			<div class="inner">
				<header class="major">
					<h3>Sed nunc ligula</h3>
				</header>
				<p>Nullam et orci eu lorem consequat tincidunt vivamus et sagittis magna sed nunc rhoncus condimentum sem. In efficitur ligula tate urna. Maecenas massa sed magna lacinia magna pellentesque lorem ipsum dolor. Nullam et orci eu lorem consequat tincidunt. Vivamus et sagittis tempus.</p>
				<ul class="actions">
					<li><a href="generic.html" class="button">Learn more</a></li>
				</ul>
			</div>
		</div>
	</section>
</section>
