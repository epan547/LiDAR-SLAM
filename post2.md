---
layout: post
title: Dec. 6, 2020
description: Blog Post 2
image: assets/images/upstairs.png
nav-menu: true
---

As we suspected, working with the physical neato onvolved debugging many more obscure bugs than the simulated version. We spent much of our time over the last couple weeks trying to connect to the neato over wifi with help from Paul. Eventually, he tracked down a fairly subtle error in the configuration files, and we were able to wirelessly connect our laptop to the robot. Since then, the neato has been surprisingly easy to work with, and we've been able to quite effectively run code from previous projects on it.

While this was a frustrating process, it was productive in that we got practice working with actual robotics hardware, which was a learning goal for both of us in this project. One very fun outcome is that we have already been able to map our house with the neato using the built in SLAM algorithm from the [particle filter](https://viahtml3.hypothes.is/proxy/https://comprobo20.github.io/assignments/robot_localization?via.client.openSidebar=1&via.client.requestConfigFromFrame.origin=https%3A%2F%2Flms.hypothes.is&via.client.requestConfigFromFrame.ancestorLevel=2&via.external_link_mode=new-tab) project (map image shown above). 

Once the robot was functional, we attempted to begin work on the graph optimization SLAM, but quickly hit another bug installing [G2opy](https://github.com/uoip/g2opy). After a great deal of fiddling with libraries and dependencies, we decided to look for a different library. We found [another](https://pypi.org/project/graphslam/) python wrapper for graph optimization which was much more recently updated, and also had robust documentation. This library was quite simple to install.

After the wifi issues, trouble with g2opy, as well as one of our two laptops dying and a couple days spent attending to gas leaks and power outages, we are behind our intended schedule for the project. Our next step will be to implement the most basic possible version of SLAM as a base to work off of. We will translate scan data based on odometry with no correction for flaws in the sensor data, and make a map out of that. This will be a non-trivial task, but we hope it will be doable by the middle of this week.
