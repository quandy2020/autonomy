

Introduction
=============

Autolink is an open source, high performance runtime framework designed specifically for autonomous driving 
scenarios. Based on a centralized computing model, it is greatly optimized for high concurrency, low latency, 
and high throughput in autonomous driving.

During the last few years of the development of autonomous driving technologies, we have learned a lot from our 
previous experience with Apollo. The industry is evolving and so is Apollo. Going forward, Apollo has already 
moved from development to productization, with volume deployments in the real world, we see the demands for the
highest level of robustness and performance. That's why we spent years building and perfecting Autolink, 
which addresses that requirements of autonomous driving solutions.


Key benefits of using Autolink:

- Accelerate development

  + Well defined task interface with data fusion
  + Array of development tools
  + Large set of sensor drivers
- Simplify deployment

  + Efficient and adaptive message communication
  + Configurable user level scheduler with resource awareness
  + Portable with fewer dependencies
- Empower your own autonomous vehicles

  + The default open source runtime framework
  + Building blocks specifically designed for autonomous driving
  + Plug and play your own AD system

.. toctree::
   :caption: QUICKSTART
   :maxdepth: 1

   Autolink_Quick_Start.md
   Autolink_Terms
   Autolink_FAQs

.. toctree::
   :caption: TUTORIALS
   :maxdepth: 2

   Autolink_API_for_Developers
   Autolink_Python_API.md
   Autolink_Developer_Tools.md

.. toctree::
   :maxdepth: 2
   :caption: ADVANCED TOPICS

   Autolink_Docker.md
   Autolink_Migration_Guide.md

.. toctree::
   :caption: API REFERENCE
   :maxdepth: 2

   api/cppapi_index
   api/pythonapi_index

