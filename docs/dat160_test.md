---
layout: default
title: DAT160
nav_order: 1
---

This is the test site for DAT160!

{% include_relative share_test/common_content.md %}

{% for staff_member in site.shared %}
  {% if staff_member.title == "install_vm" %}
  <p>{{ staff_member.content | markdownify }}</p>
  {% endif %}
{% endfor %}