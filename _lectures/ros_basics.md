---
layout: default
title: ROS Basics
nav_order: 4
---

{% for staff_member in site.shared %}
  {% if staff_member.title == "ros_basics" %}
  <p>{{ staff_member.content | markdownify }}</p>
  {% endif %}
{% endfor %}