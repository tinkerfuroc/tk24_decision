import py_trees
import py_trees_ros
import rclpy


from .PickUpTrash import createPickUpTrashTree


def main():
    # rclpy.init(args=None)

    root = createPickUpTrashTree()

    py_trees.display.render_dot_tree(root)


if __name__ == "__main__":
    main()