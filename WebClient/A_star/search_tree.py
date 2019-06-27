import board
import heapq
"""
Serch_node repressents a serach tree node, 
state : is the board the node repressents 
parent : the serach node that came before it
action : a tupple of object cordinated , and direction
"""
class Search_node:
    def __init__(self,state,parent=None,action=None,cost = 0):
        self.state = state
        self.parent = parent
        self.action = action
        self.cost = cost

    def expand_node (self):
        expanded_boards = self.state.get_next_boards()
        expanded_nodes = []
        for brd in expanded_boards :
            new_node = Search_node(brd[0],self,(brd[1],brd[2],self.cost+1))
            expanded_nodes.append(new_node)
        return expanded_nodes
 
def recursive_limited_depth_search (current_state,goal_test,limit):
    if(goal_test(current_state.state)):
        return find_solution(current_state)
    elif (limit == 0):
        return None
    else:
        next_states = current_state.expand_node()
        for state in next_states:
            result = recursive_limited_depth_search(state,goal_test,limit-1)
            if (result != None):
                return result
        return None

"""
pre: state is a search_node that has passed a goal test
post: returns the path from the intial state to state (aka the goal test)
(path is an ordered list of tupples, each repressents a object and a direction it is moved to )
"""
def find_solution(state):
    path = []
    current_state = state
    while (current_state != None):
        if (current_state.action != None):
            path.insert(0,(current_state.action))
        current_state= current_state.parent
    return path
    


def iterative_deepening(inital_state,goal_test):
    result = None
    depth=1
    while(result == None):
        result = recursive_limited_depth_search(inital_state,goal_test,depth)
        depth+=1
    return result 

def memory_depth_limited_search (current_state,goal_test,limit,explored=[]):
    explored.append(current_state.state)
    if(goal_test(current_state.state)):
        return find_solution(current_state)
    elif (limit == 0):
        return None
    else:
        next_states = current_state.expand_node()
        for state in next_states:
            if(not state.state in explored):
                result = memory_depth_limited_search(state,goal_test,limit-1,explored)
            if (result != None):
                return result
        return None

def memory_iterative_deepening(inital_state,goal_test):
    result = None
    depth=1
    while(result == None):
        result = memory_depth_limited_search(inital_state,goal_test,depth)
        depth+=1
    return result 

def Best_first_search (initial_state,goal_test,goal_board):
    current_state = initial_state
    frontier = []
    explored = set()
    counter = 0 # currently counter seperates between time added to the queue , need to solve this in a more elegant manner
    while(not goal_test(current_state.state)):
        explored.add(current_state.state)            
        next_states = current_state.expand_node()
        for state in next_states:
            if(state.state not in explored):    
                heapq.heappush(frontier,(board.unique_unordered_MD(state.state,goal_board)+state.cost,counter,state))
                counter+=1
        if (not frontier):
            return None
        current_state = heapq.heappop(frontier)[2]
    return (find_solution(current_state))