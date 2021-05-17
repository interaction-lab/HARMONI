Overview
================

Human And Robot Modular OpeN Interactions (HARMONI) is a comprehensive tool containing all the components and capabilities you need to quickly get your social interaction up and running on a robot. 

HARMONI packages the additional capabilities needed for social human-robot interaction neatly on top of ROS by wrapping and integrating several state of the art libraries in the domains of natural language understanding, dialog, object and face detection, and decision trees. HARMONI is the glue that integrates these disparate packages on top of ROS in a standardized way so you don't have to. Just `install our docker images <quickstart/Docker-Quickstart.html>`_ and `configure the capabilities you need <Configuration.html>`_ for your interaction.

HARMONI is built to be Robot-Agnostic, Modular, and Composable. By building on top of ROS and wrapping the whole platform in Docker, HARMONI should work on most hardware platforms and operating systems. By keeping to well defined interfaces with loose coupling and encapsulation HARMONI is a modular distributed system. This modularity further enables the system to be composed to suit the user's needs just through configuration files.


.. toctree::
   :maxdepth: 1
   :caption: Contents:
   :glob:

   overview/Architecture
   overview/Harmoni-and-Docker
   overview/Package-Organization
   overview/Package-List