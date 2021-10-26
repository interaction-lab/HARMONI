# HARMONI Pytree

PyTree is one among the several ways to implement behaviour trees.  
A tree in pytree is made of two elements: nodes and leaves.
Nodes can be of different nature but in general they belong to the family of composites (sequence, parallel, selector). The management of these elements it’s not something that users have to do, everything has already been done by the creators of pytree. 
Leaves come from a single element called “behaviour” and these are the components that users can create. Even if pytree made some basic behaviours available for us (you can find some of them [here](https://py-trees.readthedocs.io/en/devel/modules.html#module-py_trees.behaviours)), developers are required to create their own behaviours ([skeleton of behaviour](https://py-trees.readthedocs.io/en/devel/behaviours.html#skeleton)). We provide some leaves that represent the pytree version of clients of HARMONI.

## Setup

In order to use PyTree import the python library by running the following command:

```bash 
$ pip install py_trees
``` 
**(optional)** If you want to create and see dot_tree you must install pydot and graphviz.
For pydot run:

```bash 
$ pip install pydot
``` 
 For graphicviz run:
 
```bash 
$ sudo apt install graphviz
``` 
## Usage

The execution of a tree is done by ticking it.<br />
A tick starts from the root and then is propagated down in the tree up to leaves. Leaves will perform their tasks and in the end compute a state that will be returned and propagated back until the root.<br />
As mentioned before a tree is composed of nodes and leaves. Since all the leaves are behaviours that have their own class we suggest creating a different script for the body that will import all the behaviours that it needs (an example [here](https://py-trees.readthedocs.io/en/devel/trees.html#skeleton))
You can notice that after creating the tree all you need to do is call the function `behaviour_tree.tick_tock()` that will take care of ticking the tree. Parameters `pre_tick_handler` and `post_tick_handler` are used to lick functions that will be executed respectively before and after the tickling of the tree. In general they are used to see the status of the tree and additional information like the content of the blackboards.
We suggest adding a special element of the family of visitors that is called *snapshot visitor*. This component is used combined with `display.unicode_tree` to also show the statue of the visited element in the tree.<br />
You can add it by adding the following lines in the declaration of the behaviour tree

```  
snapshot_visitor = py_trees.visitors.SnapshotVisitor()
behaviour_tree.visitors.append(snapshot_visitor)
```
and then adding in the parameter `visited` in `dispaly.unicode_tree`  like that

```  
py_trees.display.unicode_tree(
            root=behaviour_tree.root,
            visited=snapshot_visitor.visited)
```
You can also decide to manually tick the tree by running `behaviour_tree.tick()` instead of `tick_tock()` function. 
PyTree offers a lot of components and useful tools that developers can use in their trees but here we will mention **composites** and **blackboards**. We recommend reading the documentation for understand all the concepts:   [https://py-trees.readthedocs.io/en/devel/behaviours.html](https://py-trees.readthedocs.io/en/devel/behaviours.html)

#### composites

Composites are responsible for directing the path traced through the tree on a given tick (execution). They are the factories (Sequences and Parallels) and decision makers (Selectors) of a behaviour tree.
Composite behaviours typically manage children and apply some logic to the way they execute and return a result, but generally don’t do anything themselves. Perform the checks or actions you need to do in the non-composite behaviours.
[https://py-trees.readthedocs.io/en/devel/composites.html#composites](https://py-trees.readthedocs.io/en/devel/composites.html#composites)

#### Blackboards

Blackboards are not a necessary component of behaviour tree implementations, but are nonetheless, a fairly common mechanism for sharing data between behaviours in the tree.
[https://py-trees.readthedocs.io/en/devel/blackboards.html#blackboards](https://py-trees.readthedocs.io/en/devel/blackboards.html#blackboards)

## Testing

Here are the steps that you can follow in order to run a demo of pytree in HARMONI. This demo show how work a small three with just two behaviours: microphone and stt.

1.  Create a valid google credential of using stt service: [https://cloud.google.com/speech-to-text/docs/before-you-begin#setting_up_your_google_cloud_platform_project](https://cloud.google.com/speech-to-text/docs/before-you-begin#setting_up_your_google_cloud_platform_project)
2.  Run the following command in the container to start the demo:

     ```  bash
     roslaunch harmoni_pytree mic_and_stt.launch
     ```

You will see some information about the structure of the tree and the way in which pytree engines tick the tree. Some blackboards are used to show the results of the leaves and in particular in backboards stt/result shows the result of the speech to text services.