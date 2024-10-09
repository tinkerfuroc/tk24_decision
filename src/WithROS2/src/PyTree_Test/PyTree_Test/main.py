import py_trees as pytree
from .custom_nodes_ros import Bt_node_scanfor, GotoObject

import py_trees_ros as pytree_ros
import rclpy

import time

def main():
    rclpy.init(args=None)

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
    scan_for_fridge = Bt_node_scanfor(name="Scan for fridge", goal_object="Fridge")
    # a behavior that goes to the fridge
    goto_fridge = GotoObject(name="Go to fridge", goal_object="Fridge")

    end_node = pytree.behaviours.Running(name="All Finished!")

    # add the nodes to the root in a sequence
    root.add_children([goto_start_c, scan_for_fridge, goto_fridge, end_node])
    pytree.display.render_dot_tree(root)

    # create a behavior tree with the root node
    tree = pytree_ros.trees.BehaviourTree(root)
    # call setup for the tree
    tree.setup(node_name="root_node", timeout=15)

    # function for display the tree to standard output
    def print_tree(tree):
        print(pytree.display.unicode_tree(root=tree.root, show_status=True))
    
    
    tree.tick_tock(period_ms=1000.0,post_tick_handler=print_tree)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()

    # try:
    #     # while the root is not SUCCESS or FAILURE
    #     while (root.status is not pytree.common.Status.SUCCESS) and (root.status is not pytree.common.Status.FAILURE):
    #         # tick the tree, print its status, and wait for 500 miliseconds
    #         tree.tick()
    #         print_tree(tree)
    #         time.sleep(0.5)
        
    #     # print the status afterwards
    #     print(root.status)
    # # allow for force exit from the command-line    
    # except KeyboardInterrupt:
    #     tree.interrupt()