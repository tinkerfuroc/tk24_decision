import py_trees
import py_trees_ros
import rclpy


from .PickUpTrash.PickUpTrash import createPickUpTrashTree


def draw_pick_up_trash():
    root = createPickUpTrashTree()
    py_trees.display.render_dot_tree(root, with_blackboard_variables=True)


def pick_up_trash():
    rclpy.init(args=None)

    root = createPickUpTrashTree()
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name="root_node", timeout=15)

    # function for display the tree to standard output
    def print_tree(tree):
        print(py_trees.display.unicode_tree(root=tree.root, show_status=True))
    
    
    tree.tick_tock(period_ms=500.0,post_tick_handler=print_tree)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == "__main__":
    pick_up_trash()