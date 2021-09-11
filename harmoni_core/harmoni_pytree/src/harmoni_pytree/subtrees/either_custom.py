import operator
import typing

from py_trees import behaviour
from py_trees import behaviours
from py_trees import blackboard
from py_trees import common
from py_trees import composites

def either_or(
    conditions: typing.List[common.ComparisonExpression],
    subtrees: typing.List[behaviour.Behaviour],
    preemptible: bool = False,
    name="Either Or",
    namespace: typing.Optional[str]=None
) -> behaviour.Behaviour:
    """
    Often you need a kind of selector that doesn't implement prioritisations, i.e.
    you would like different paths to be selected on a first-come, first-served basis.

    .. code-block:: python

        task_one = py_trees.behaviours.TickCounter(name="Subtree 1", duration=2)
        task_two = py_trees.behaviours.TickCounter(name="Subtree 2", duration=2)
        either_or = py_trees.idioms.either_or(
            name="EitherOr",
            conditions=[
                py_trees.common.ComparisonExpression("joystick_one", "enabled", operator.eq),
                py_trees.common.ComparisonExpression("joystick_two", "enabled", operator.eq),
            ],
            subtrees=[task_one, task_two],
            namespace="either_or",
        )

    .. graphviz:: dot/idiom-either-or.dot
        :align: center
        :caption: Idiom - Either Or

    Up front is an XOR conditional check which locks in the result on the blackboard
    under the specified namespace. Locking the result in permits the conditional
    variables to vary in future ticks without interrupting the execution of the
    chosen subtree (an example of a conditional variable may be one that has
    registered joystick button presses).

    Once the result is locked in, the relevant subtree is activated beneath the
    selector. The children of the selector are, from left to right, not in any
    order of priority since the previous xor choice has been locked in and isn't
    revisited until the subtree executes to completion. Only one
    may be active and it cannot be interrupted by the others.

    The only means of interrupting the execution is via a higher priority in the
    tree that this idiom is embedded in.

    Args:
        conditions: list of triggers that ultimately select the subtree to enable
        subtrees: list of subtrees to tick from in the either_or operation
        name: the name to use for this idiom's root behaviour
        preemptible: whether the subtrees may preempt (interrupt) each other
        namespace: this idiom's private variables will be put behind this namespace

    Raises:
        ValueError if the number of conditions does not match the number of subtrees

    If no namespace is provided, a unique one is derived from the idiom's name.

    .. seealso:: :ref:`py-trees-demo-either-or <py-trees-demo-either-or-program>`

    .. todo:: a version for which other subtrees can preempt (in an unprioritised manner) the active branch
    """
    if len(conditions) != len(subtrees):
        raise ValueError("Must be the same number of conditions as subtrees [{} != {}]".format(
            len(conditions), len(subtrees))
        )
    root = composites.Sequence(name=name)
    configured_namespace: str = namespace if namespace is not None else \
        blackboard.Blackboard.separator + name.lower().replace("-", "_").replace(" ", "_") + \
        blackboard.Blackboard.separator + str(root.id).replace("-", "_").replace(" ", "_") + \
        blackboard.Blackboard.separator + "conditions"
    xor = behaviours.CheckBlackboardVariableValues(
        name="XOR",
        checks=conditions,
        operator=operator.xor,
        namespace=configured_namespace
    )
    chooser = composites.Selector(name="Chooser")
    if preemptible:
        for counter in range(1, len(conditions) + 1):
            sequence = composites.Sequence(name="Option {}".format(str(counter)))
            variable_name = configured_namespace + blackboard.Blackboard.separator + str(counter)
            disabled = behaviours.CheckBlackboardVariableValue(
                name="Enabled?",
                check=common.ComparisonExpression(
                    variable=variable_name,
                    value=True,
                    operator=operator.eq
                )
            )
            sequence.add_children([disabled, subtrees[counter - 1]])
            chooser.add_child(sequence)
        root.add_children([xor, chooser])
    else:
        for counter in range(1, len(conditions) + 1):
            sequence = composites.Sequence(name="Option {}".format(str(counter)))
            variable_name = configured_namespace + blackboard.Blackboard.separator + str(counter)
            disabled = behaviours.CheckBlackboardVariableValue(
                name="Enabled?",
                check=common.ComparisonExpression(
                    variable=variable_name,
                    value=True,
                    operator=operator.eq
                )
            )
            sequence.add_children([disabled, subtrees[counter - 1]])
            chooser.add_child(sequence)
        root.add_children([xor, chooser])
    return root