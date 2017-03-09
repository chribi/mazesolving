UP = 0
RIGHT = 1
DOWN = 2
LEFT = 3

def connect(node1, outDirection1, node2, outDirection2, corner):
    """
    Connect node1 to node2.
    The path starts at node1 in direction outDirection1.
    It ends at node2 from direction outDirection2.
    When connecting two temporary nodes the two connected nodes will be merged 
    into the first node.
    This ensures that two temporary nodes will NEVER be connected.
    When connecting a temporary node with a final node, the temporary node
    will NOT appear as a neighbour of the  final node.  Final nodes will ONLY
    connect to other final nodes.
    corner should be != None if there is an corner in the connection path
    that has to be included in the path.

    """
    if node1 == None or node2 == None:
        return False
    
    if not (node1.IsFinal or node2.IsFinal):
        # Two temporary nodes will be merged into node1
        # node2 is temporary, so it has paths RIGHT and DOWN,
        # therefore 3-outDirection2 is the other outgoing direction of node2
        node1.Neighbours[outDirection1] = node2.Neighbours[3 - outDirection2]
        # Update path
        path = node1.Paths[outDirection1]
        if corner != None:
            path.append(corner)
        path += node2.Paths[outDirection2][::-1]
        path.append(node2.Position)
        path += node2.Paths[3 - outDirection2]

        # resolve the node if possible
        resolve_temporary_node(node1)
        return True
    else:
        # at least one node is final => create connections to 
        # each final node.
        if node1.IsFinal:
            node2.Neighbours[outDirection2] = node1
            path = node2.Paths[outDirection2] 
            if corner != None:
                path.append(corner)
            path += node1.Paths[outDirection1][::-1]

        if node2.IsFinal:
            node1.Neighbours[outDirection1] = node2
            path = node1.Paths[outDirection1]
            if corner != None:
                path.append(corner)
            path += node2.Paths[outDirection2][::-1]
        # resolve temporary nodes if possible
        if not node1.IsFinal:
            resolve_temporary_node(node1)
        if not node2.IsFinal:
            resolve_temporary_node(node2)
        return False

def resolve_temporary_node(node):
    """
    Replaces a temporary node that is connected with two final nodes
    with a connection between the two connected nodes.
    
    """
    if node != None and not node.IsFinal:
        rightNeighbour = node.Neighbours[RIGHT]
        pathRight = node.Paths[RIGHT]
        if len(pathRight) == 0:
            pathRightLast = node.Position
        else:
            pathRightLast = pathRight[-1]
        downNeighbour = node.Neighbours[DOWN]
        pathDown = node.Paths[DOWN]
        if len(pathDown) == 0:
            pathDownLast = node.Position
        else:
            pathDownLast = pathDown[-1]

        if rightNeighbour != None and downNeighbour != None:
            rightOut = relative_position(pathRightLast, rightNeighbour.Position)
            downOut = relative_position(pathDownLast, downNeighbour.Position)
            path = pathRight[::-1]
            path.append(node.Position)
            path += pathDown
            rightNeighbour.Neighbours[rightOut] = downNeighbour
            rightNeighbour.Paths[rightOut] = path
            downNeighbour.Neighbours[downOut] = rightNeighbour
            downNeighbour.Paths[downOut] = path[::-1]

def relative_position(pos1, pos2):
    if pos1[0] < pos2[0]:
        return UP
    elif pos1[0] > pos2[0]:
        return DOWN
    elif pos1[1] < pos2[1]:
        return LEFT
    else:
        return RIGHT

class OptimizedMaze:
    """
    OptimizedMaze that will only have nodes for intersections.  This reduces
    the number of nodes significantly.  It also removes dead ends from the graph.

    The graph generation will still work with a single pass over the image.
    """
    class Node:
        def __init__(self, position, final):
            # Whether this is a final or a temporary node
            self.IsFinal = final
            # position (y, x) in the maze
            self.Position = position
            # The next node in all directions, starting clockwise from the up direction.
            self.Neighbours = [None, None, None, None]
            # self.Paths[direction] is the the path to self.Neighbours[direction]
            # given as list of corner points, starting from this node
            self.Paths = [[], [], [], []]

    def __init__(self, image):
        width = image.size[0]
        height = image.size[1]
        data = list(image.getdata(0))

        self.start = None
        self.end = None

        # Top row buffer
        topnodes = [None] * width
        topnodes_out = [DOWN] * width
        count = 0

        # Start row
        for x in range (1, width - 1):
            if data[x] > 0:
                self.start = OptimizedMaze.Node((0,x), True)
                topnodes[x] = self.start
                count += 1
                break

        for y in range (1, height - 1):
            #print ("row", str(y)) # Uncomment this line to keep a track of row progress

            rowoffset = y * width
            rowaboveoffset = rowoffset - width
            rowbelowoffset = rowoffset + width

            # Initialise previous, current and next values
            left = False
            current = False
            right = data[rowoffset + 1] > 0

            leftnode = None
            left_out = RIGHT

            for x in range (1, width - 1):
                left = current
                current = right
                right = data[rowoffset + x + 1] > 0

                current_position = (y, x)

                if current == False:
                    # ON WALL - No action
                    continue

                up = data[rowaboveoffset + x] > 0
                down = data[rowbelowoffset + x] > 0

                numNeighbours = left + right + up + down

                # the node an "up" path would lead to from the current position
                topnode = topnodes[x]

                if numNeighbours == 1:
                    # We are in a dead end
                    # => Set topnodes[x] and leftnode to None so
                    # that no connection to the dead end will be created
                    topnodes[x] = None
                    leftnode = None
                elif numNeighbours == 2:
                    # We are in a corridor
                    if left and up and leftnode != None and topnode != None:
                        merged = connect(leftnode, left_out, topnode, topnodes_out[x], current_position)
                        if merged:
                            # if we just merged topnode into leftnode, we may need
                            # to replace topnode somewhere in topnodes with leftnode
                            for i in range(0, width):
                                # TODO this should be possible without traversing topnodes
                                if topnodes[i] == topnode:
                                    topnodes[i] = leftnode
                                    topnodes_out[i] = left_out
                            
                    elif left and down:
                        topnodes[x] = leftnode
                        topnodes_out[x] = left_out
                        if leftnode != None:
                            leftnode.Paths[left_out].append(current_position)
                    elif up and right:
                        leftnode = topnode
                        left_out = topnodes_out[x]
                        if topnode != None:
                            topnode.Paths[topnodes_out[x]].append(current_position)
                    elif right and down:
                        # have a corner which ways pointing in unexplored space 
                        # => Create a temporary corner Node
                        leftnode = OptimizedMaze.Node(current_position, False)
                        topnodes[x] = leftnode
                        topnodes_out[x] = DOWN
                        left_out = RIGHT
                elif numNeighbours >= 3:
                    # We are at an intersection => Create a node
                    node = OptimizedMaze.Node(current_position, True)
                    count += 1
                    if left and leftnode != None: 
                        connect(leftnode, left_out, node, LEFT, None)
                    if up and topnode != None:
                        connect(topnode, topnodes_out[x], node, UP, None)
                    if right:
                        leftnode = node
                        left_out = RIGHT
                    if down:
                        topnodes[x] = node
                        topnodes_out[x] = DOWN

        # End row
        rowoffset = (height - 1) * width
        for x in range (1, width - 1):
            if data[rowoffset + x] > 0:
                self.end = OptimizedMaze.Node((height - 1,x), True)
                count += 1
                topnode = topnodes[x]
                connect(topnode, topnodes_out[x], self.end, UP, None)
                break

        self.count = count
        self.width = width
        self.height = height

def expand_path(path):
    """
    Expand a path of OptimizedMaze.Nodes.  This is needed for drawing the path,
    as the drawing algorithm can only work with vertical or horizontal line segments. 
    """
    current = path[0]
    expanded = [current.Position]
    for nxt in path:
        # skip first
        if nxt == current:
            continue
        out_direction = current.Neighbours.index(nxt)
        expanded += current.Paths[out_direction]
        expanded.append(nxt.Position)
        current = nxt
    return expanded

def print_node(node):
    nodepos = lambda n: n.Position if n != None else "__"
    print (node.Position, [nodepos(n) for n in node.Neighbours])
