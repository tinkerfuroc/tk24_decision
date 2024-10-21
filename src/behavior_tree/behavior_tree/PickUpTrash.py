import py_trees as pytree
import py_trees_ros as pytree_ros

def createPickUpTrashTree() -> pytree.composites.Sequence:
    root = pytree.composites.Sequence(name="Pick Up Trash Root", memory=True)


    return root