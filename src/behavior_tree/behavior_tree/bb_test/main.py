import py_trees as pytree
from .custom_nodes import ScanFor, GotoObject

import time

if __name__ == "__main__":
    # root of the tree
    root = pytree.composites.Sequence(name="Mock Root", memory=True)

    write_read = pytree.composites.Sequence(name="Write Read", memory=True)
    
    # a behavior that scans for the fridge
    scan_for_fridge = ScanFor(name="Scan for fridge", goal_object="Fridge")
    write_read.add_child(scan_for_fridge)
    # a behavior that goes to the fridge
    goto_fridge = GotoObject(name="Go to fridge", goal_object="Fridge")
    write_read.add_child(goto_fridge)

    root.add_child(pytree.decorators.Repeat("repeat", write_read, 10))

    pytree.display.render_dot_tree(root)

    # create a behavior tree with the root node
    tree = pytree.trees.BehaviourTree(root)
    # call setup for the tree
    tree.setup(timeout=15)

    # function for display the tree to standard output
    def print_tree(tree):
        print(pytree.display.unicode_tree(root=tree.root, show_status=True))
        print(pytree.display.unicode_blackboard())

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

def createBBTestTree():
    root = pytree.composites.Sequence(name="Mock Root", memory=True)

    write_read = pytree.composites.Sequence(name="Write Read", memory=True)
    
    # a behavior that scans for the fridge
    scan_for_fridge = ScanFor(name="Scan for fridge", goal_object="Fridge")
    write_read.add_child(scan_for_fridge)
    # a behavior that goes to the fridge
    goto_fridge = GotoObject(name="Go to fridge", goal_object="Fridge")
    write_read.add_child(goto_fridge)

    root.add_child(pytree.decorators.Repeat("repeat", write_read, 10))

    return root