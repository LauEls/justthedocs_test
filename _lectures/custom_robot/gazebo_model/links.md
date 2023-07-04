---
layout: default
title: Link definition
parent: Building a Gazebo Model
grand_parent: Build a Custom Robot
nav_order: 1
---

{% for shared_site in site.shared %}
  {% if shared_site.title == "build_custom_robot/building_gazebo_model/links" %}
  <p>{{ shared_site.content | markdownify }}</p>
  {% endif %}
{% endfor %}