# Running Individual Packages

## Running a Package in HARMONI
If you want to run a single package, you can directly launch the node from the command line as shown in the Launching a Service page.

### Playing a Pattern

In harmoni we seek to provide a single point of truth for the configuration of the routers and services. Rather than generating multiple launch files for each configuration set, we use the python api for roslaunch to dynamically launch files based on the current configuration. We use launcher.launch to start the launcher.py node, which will start up the routers and services listed in the configuration.yaml file.


## Running Packages Alone

Todo: Show how to write a client for a given service.


