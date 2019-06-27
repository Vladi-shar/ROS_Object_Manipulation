import random
class Board:
    #the grids has y axis inverted (like a two dimensional array) :
    #  (0,0),(1,0),(2,0)......
    #  (0,1),(1,1),(2,1)......
    #  (0,2),(1,2),(2,2)...... 
    #    .     .     .
    #    .     .     .
    #    .     .     .
    def __init__ (self,x_length,y_length,object_list=None):
        self.x_length=x_length
        self.y_length=y_length
        self.object_list=[]
        # possible intial list of objects  
        if(object_list!= None):   
            self.object_list=object_list
# may be a need to change object represantaition to a dictionary instead of tupple to promote more readabilty
    def __eq__ (self,other_board):
        if(len(self.object_list) != len(other_board.object_list )):
            return False
        for obj in self.object_list:
            if (obj not in other_board.object_list):
                return False
        return True  
    def __hash__(self):
        ordered_list  = self.object_list.sort()
        return hash(str(ordered_list))
    def add_object (self,x,y):           
        """add a new object to the board

        pre: x<x_length , y<y_length . cells[(x,y)]= false
        post: cells[(x,y)] = true
        """
        if (x>=self.x_length or y>= self.y_length):
            raise Exception('out of bounds:('+str(x)+','+str(y)+')')
        if((x,y) in self.object_list):
            raise Exception('Object already exists in Cell:('+str(x)+','+str(y)+')')
        self.object_list.append((x,y))

    def get_next_boards (self):
        """get the next possible boards(states)
        
        pre: none 
        post: returns a list of all possible boards after one move
        """  
        boards_with_actions = [] # each entery consists of the tupple (board,object,action)
        #add functions
        for obj in self.object_list:
            possible_moves={'up':True,'down_left':True,'down_right':True,'left':True,'right':True}
            if(obj[0]== 0):
                possible_moves['left']=False
            if(obj[0]==self.x_length-1):
                possible_moves['right']=False
            if(obj[1]==0) :
                possible_moves['up']=False
            if(obj[1]==self.y_length-1):
                possible_moves['down_left']=False 
                possible_moves['down_right']=False
            for other_obj in self.object_list:
                #break condition for saving time
                if(not True in possible_moves.values()):
                    break
                self.check_left_right_moves(obj,other_obj,possible_moves)
                self.check_up_move(obj,other_obj,possible_moves)
                self.check_down_move(obj,other_obj,possible_moves)    
                              
            #add a list with each possible move done
            if(possible_moves['up']==True):
                altered_object_list=self.object_list.copy()
                altered_object_list.remove(obj)
                altered_object_list.append((obj[0],obj[1]-1))
                boards_with_actions.append((Board(self.x_length,self.y_length,altered_object_list),obj,'up'))
            if(possible_moves['down_left']==True):
                altered_object_list=self.object_list.copy()
                altered_object_list.remove(obj)
                altered_object_list.append((obj[0],obj[1]+1))
                boards_with_actions.append((Board(self.x_length,self.y_length,altered_object_list),obj,'down_left'))
            if( possible_moves['down_right']==True):
                altered_object_list=self.object_list.copy()
                altered_object_list.remove(obj)
                altered_object_list.append((obj[0],obj[1]+1))
                boards_with_actions.append((Board(self.x_length,self.y_length,altered_object_list),obj,'down_right'))
            if(possible_moves['left']==True):
                altered_object_list=self.object_list.copy()
                altered_object_list.remove(obj)
                altered_object_list.append((obj[0]-1,obj[1]))
                boards_with_actions.append((Board(self.x_length,self.y_length,altered_object_list),obj,'left'))
            if(possible_moves['right']==True):
                altered_object_list=self.object_list.copy()
                altered_object_list.remove(obj)
                altered_object_list.append((obj[0]+1,obj[1]))
                boards_with_actions.append((Board(self.x_length,self.y_length,altered_object_list),obj,'right'))
        return boards_with_actions

    def check_left_right_moves (self,current_object, other_object,possible_moves):
        #if the other object is in left side  and is also closer to the "start" of the table
        if(current_object[0]==other_object[0]+1 and other_object[1]>=current_object[1]):
            if (other_object[1]==current_object[1]):#if there is an  object just to the left of the current object
                possible_moves['left']=False
            possible_moves['right']=False
        #if the other object is in right side  and is also closer to the "start" of the table
        if(current_object[0]==other_object[0]-1 and other_object[1]>=current_object[1]):
            if (other_object[1]==current_object[1]):
                possible_moves['right']=False   #if there is an  object just to the right of the current object
            possible_moves['left']=False

    def check_up_move(self,current_object, other_object,possible_moves):
        if (current_object[0]== other_object[0] ):
            if (current_object[1]<other_object[1]):
                possible_moves['up']=False
            elif  (current_object[1]==other_object[1]+1):
                possible_moves['up']=False

    def check_down_move(self,current_object, other_object,possible_moves):
        if(current_object[0]==other_object[0]+1 and other_object[1]>=current_object[1]-1):
            possible_moves['down_left']=False
        if(current_object[0]==other_object[0]-1 and other_object[1]>=current_object[1]-1):
            possible_moves['down_right']=False  
        if (current_object[0]== other_object[0]):
            if  (current_object[1]==other_object[1]+1):
                possible_moves['down_left']=False
                possible_moves['down_right']=False
            if  (current_object[1]==other_object[1]-1):
                possible_moves['down_left']=False
                possible_moves['down_right']=False 


    
    def board_to_string(self):
        string =''
        for i in range(self.y_length):
            string+='\n'
            for j in range(self.x_length):
                string+= ('0 ','x ' )[ (j,i) in self.object_list]
        return string
       
def random_board(x_length,y_length,objects_number):
    objects_left_to_generate = objects_number
    object_list = []
    while (objects_left_to_generate!=0):
        x_cord= random.randint(0,x_length-1)
        y_cord= random.randint(0,x_length-1)
        if(not (x_cord,y_cord) in object_list):
            objects_left_to_generate-=1
            object_list.append((x_cord,y_cord))
    brd=Board(x_length,y_length, object_list)
    return brd 
"""
pre cond = the two boards have thesame dimensions (x and y)
post cond : returns the sum of distances from each object in brd1 to it's closest object in brd2
time complexity : O(number_of_objects^2) (can be improved)
"""
def closest_manhattan_distance(brd1,brd2):
    sum = 0
    closest = -1
    for obj1 in brd1.object_list:
        for obj2 in brd2.object_list:
            if (closest == -1):
                closest = manhattan_distance(obj1,obj2)
            closest= min(closest,manhattan_distance(obj1,obj2))
        sum += closest
        closest = -1
    return sum
def unique_unordered_MD(brd1,brd2):
    list2 = brd2.object_list.copy()
    sum = 0
    closest = -1
    for obj1 in brd1.object_list:
        for obj2 in list2:
            if (closest == -1):
                closest = manhattan_distance(obj1,obj2)
                closest_obj = obj2
            if(closest > manhattan_distance(obj1,obj2)):
                closest = manhattan_distance(obj1,obj2)
                closest_obj = obj2 
        list2.remove(closest_obj)
        sum+= closest
        closest = -1
    return sum
def manhattan_distance(first_pos,second_pos):
    if (first_pos[0]>= second_pos[0]):
        x_dist = first_pos[0] - second_pos[0]
    else:
        x_dist = second_pos[0]-first_pos[0] 
    if (first_pos[1]>= second_pos[1]):
        y_dist = first_pos[1] - second_pos[1]
    else:
        y_dist = second_pos[1]-first_pos[1] 
    return (x_dist+y_dist)


        
        





     
