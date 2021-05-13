# Contributing

If you would like to submit to the HARMONI official repository, fork the develop branch, make your changes, and then submit a pull request to bring the new content from your fork into the official repository. The maintainers will review your update and let you know if there are any issues.

## Developing in HARMONI
For configuring git in the harmoni_core docker container you should create `.gitconfig` as a file instead of a directory:
~~~
$ cd ~
$ umount ~/.gitconfig
$ rm -rf .gitconfig
$ cat .gitconfig
~~~
Then press Ctrl-X to exit the file created. Then you can config your github in the container:
~~~
$ git config --global user.email "$EMAIL"
$ git config --global user.name "$NAME"
~~~
Now you can start developing.
Remember to push your contribution.

### Create a new package
If you are creating a new package, go to the terminal of the repo where you want to create your package, and run:
~~~
catkin create pkg $NAME
~~~

Most of the time however, you will be adding a node with complementary capability to an existing package. This node will be a Harmoni Unit and must abide by the conventions of the Harmoni Unit in order to work with the rest of HARMONI. The easiest way to create a new Harmoni Unit is by starting either from copying an existing node and editing it or by starting from the Harmoni Template we provide.