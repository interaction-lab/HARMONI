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
    
    if len(conditions) != len(subtrees):
        raise ValueError("Must be the same number of conditions as subtrees [{} != {}]".format(
            len(conditions), len(subtrees))
        )
    
    if preemptible:
        root = composites.Sequence(name=name, memory=False)
    else
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