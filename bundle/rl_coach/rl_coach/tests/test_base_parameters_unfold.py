import json
import sys

import pytest

from rl_coach.base_parameters import Parameters, unfold_dict_or_list


class _Node:
    def __init__(self, name):
        self.name = name
        self.peer = None


class _Holder(Parameters):
    def __init__(self):
        super().__init__()
        self.cls = None
        self.module = None
        self.chain = None


@pytest.mark.unit_test
def test_unfold_reference_cycle_terminates():
    a, b = _Node("a"), _Node("b")
    a.peer, b.peer = b, a
    out = unfold_dict_or_list({"a": a})
    assert out["a"]["name"] == "a"
    assert out["a"]["peer"]["name"] == "b"
    assert out["a"]["peer"]["peer"].startswith("<cycle")


@pytest.mark.unit_test
def test_unfold_class_and_module_values_are_repr():
    holder = _Holder()
    holder.cls = _Node
    holder.module = json
    text = str(holder)  # Parameters.__str__ -> json.dumps(unfold_dict_or_list(...))
    assert "_Node" in text and "module 'json'" in text


@pytest.mark.unit_test
def test_unfold_deep_chain_ending_in_class_does_not_raise():
    # A chain deeper than the recursion limit whose last element references a class: before the
    # fix the RecursionError fallback wrote '__class__' into the class's mappingproxy.
    head = node = _Node(0)
    for i in range(1, sys.getrecursionlimit() + 100):
        node.peer = _Node(i)
        node = node.peer
    node.peer = _Node
    unfold_dict_or_list({"chain": head})
