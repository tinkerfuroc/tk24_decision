import py_trees as pytree
from custom_nodes import ScanFor, GotoObject

import time

if __name__ == "__main__":
    # root of the tree
    root = pytree.composites.Sequence(name="Mock Root", memory=True)
    
    # checks whether is at starting position, always returns failure
    at_start = pytree.behaviours.Failure(name="At Starting pose")
    # the actual execution of going to the starting position, always returns success (assumes navigation successful)
    goto_start = pytree.behaviours.Success(name="goto:starting position")
    # a priority composite that returns success once one of its child returns success
    # the `at_start` acts as a guard (if it was successful, then `goto_start` would not be executed)
    goto_start_c = pytree.composites.Selector(name="Go to starting position", memory=True, children=[at_start, goto_start])
    
    # a behavior that scans for the fridge
    scan_for_fridge = ScanFor(name="Scan for fridge", goal_object="Fridge")
    # a behavior that goes to the fridge
    goto_fridge = GotoObject(name="Go to fridge", goal_object="Fridge")

    # add the nodes to the root in a sequence
    root.add_children([goto_start_c, scan_for_fridge, goto_fridge])
    pytree.display.render_dot_tree(root)

    # create a behavior tree with the root node
    tree = pytree.trees.BehaviourTree(root)
    # call setup for the tree
    tree.setup(timeout=15)

    # function for display the tree to standard output
    def print_tree(tree):
        print(pytree.display.unicode_tree(root=tree.root, show_status=True))

    try:
        # while the root is not SUCCESS or FAILURE
        while (root.status is not pytree.common.Status.SUCCESS) and (root.status is not pytree.common.Status.FAILURE):
            # tick the tree, print its status, and wait for 500 miliseconds
            tree.tick()
            print_tree(tree)
            time.sleep(0.5)
        
        # print the status afterwards
        print(root.status)
    # allow for force exit from the command-line    
    except KeyboardInterrupt:
        tree.interrupt()