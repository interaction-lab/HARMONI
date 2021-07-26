# HARMONI Face

Harmoni provides a wrapper on the cordial face, which is capable of expressing speech and emotion.

We provide a fork of the face of [CoRDial](https://github.com/ndennler/cordial-public) which was created by the Interaction Lab. Although it started as one face, we have implemented separate services for the eyes and mouth, allowing them to be controlled independently.

![packages](../images/screen_demo.png)

## Usage

## Parameters

## Testing
The face can be tested by running `rostest harmoni_face face.test`. Once this command is run, the top of the output in the terminal will read
```
Starting up http-server, serving ./
Available on:
```
and then three links will be underneath. The second link must be opened in a browser in order for the test to pass. The webpage should display an animation of the QT's face. This test checks if the face is properly displayed and will fail if the link is not opened or the face is not properly displayed.


## References
[Documentation](https://harmoni.readthedocs.io/en/latest/packages/harmoni_face.html)