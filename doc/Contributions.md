# Contributing

HARMONI is intended to serve the needs of the HRI community. We hope that HARMONI currently supports the basic needs of most HRI developers but we acknowledge there will always be room for new capabilities and improvements. As you use HARMONI for your own work we hope you will contribute back the new capabilities and robots you work with for the good of the whole community.
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

## Pointing to Studies

*When configuring HARMONI for your own research studies, you may want to share your modifications with others. This is how.*

### External Studies

*For anyone not on the HARMONI team*

TLDR:

1. Fork the repository
2. Provide those interested in your study with a link to your fork. Optionally: submit a pull request for your changes to the main HARMONI repository; be sure to note the study title.
3. Notify the HARMONI team! We would be glad to feature your work in the documentation.

If you are not part of the core HARMONI team which has push permission to the repository, or you have made substantial changes to code which has not been pulled in, or you just want to be absolutely sure your code will always be accessible, **fork the repository**. 

After you make your own fork, make whatever changes you need and provide people learning about your study with a link to your fork. If you believe the changes you have made will improve HARMONI, feel free to submit a pull request for the official repository. 

Be sure to notify the HARMONI team (feel free to make a new issue) of your study and its title so that they can tag your last commit in the repo once your changes are pulled in. Even if you are not pulling your content in the main repo, still let a HARMONI maintainer know! We would be glad to include a link to your study in the documentation.

### HARMONI Team

For anyone who has permission to make branches on the HARMONI repo, you can include your study by:

1. Making a branch off of develop
2. Doing all your work on it
3. Tagging the last commit of your branch and pushing that tag. 
4. If you have made changes to more than just configuration files, also submit a pull request and get your code merged into the repo. 

To tag and push a commit, first have that commit checked out (`git checkout branchname` will select the latest commit for that branch), then:

```bash
git tag mystudytitle
git push origin mystudytitle
```