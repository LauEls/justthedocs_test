---
layout: default
title: Build a Custom Robot
nav_order: 5
has_children: true
---

{% for shared_site in site.shared %}
  {% if shared_site.title == "build_custom_robot" %}
  <p>{{ shared_site.content | markdownify }}</p>
  {% endif %}
{% endfor %}