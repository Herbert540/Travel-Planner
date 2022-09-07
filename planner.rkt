#lang dssl2

# Final project: Trip Planner
let eight_principles = ["Know your rights.",
"Acknowledge your sources.",
"Protect your work.",
"Avoid suspicion.",
"Do your own work.",
"Never falsify a record or permit another person to do so.",
"Never fabricate data, citations, or experimental results.",
"Always tell the truth when discussing your work with your instructor."]
import cons
import sbox_hash
import  'project-lib/dictionaries.rkt'
import 'project-lib/binheap.rkt'
import 'project-lib/graph.rkt'
import 'project-lib/stack-queue.rkt'



### Basic Types ###

#  - Latitudes and longitudes are numbers:
let Lat?  = num?
let Lon?  = num?

#  - Point-of-interest categories and names are strings:
let Cat?  = str?
let Name? = str?

### Raw Entity Types ###

#  - Raw positions are 2-element vectors with a latitude and a longitude
let RawPos? = TupC[Lat?, Lon?]

#  - Raw road segments are 4-element vectors with the latitude and
#    longitude of their first endpoint, then the latitude and longitude
#    of their second endpoint
let RawSeg? = TupC[Lat?, Lon?, Lat?, Lon?]

#  - Raw points-of-interest are 4-element vectors with a latitude, a
#    longitude, a point-of-interest category, and a name
let RawPOI? = TupC[Lat?, Lon?, Cat?, Name?]

### Contract Helpers ###

# ListC[T] is a list of `T`s (linear time):
let ListC = Cons.ListC
# List of unspecified element type (constant time):
let List? = Cons.list?


struct Position:
    let pos: RawPos?
    
    let poi
    
    
def make_poi(position, interest ):
    if [position.pos] != [interest.x, interest.y]: error('something wrong here')
    
    position.poi? = True
    position.poi? = interest
    
    
def distance_squ(pos1:RawPos?, pos2: RawPos?):
    return ((pos1[0]-pos2[0])*(pos1[0]-pos2[0])) + ((pos1[1]-pos2[1])*(pos1[1]-pos2[1])) 

interface TRIP_PLANNER:

    # Returns the positions of all the points-of-interest that belong to
    # the given category.
    def locate_all(
            self,
            dst_cat:  Cat?           # point-of-interest category
        )   ->        ListC[RawPos?] # positions of the POIs

    # Returns the shortest route, if any, from the given source position
    # to the point-of-interest with the given name.
    def plan_route(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_name: Name?          # name of goal
        )   ->        ListC[RawPos?] # path to goal

    # Finds no more than `n` points-of-interest of the given category
    # nearest to the source position.
    def find_nearby(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_cat:  Cat?,          # point-of-interest category
            n:        nat?           # maximum number of results
        )   ->        ListC[RawPOI?] # list of nearby POIs


class TripPlanner (TRIP_PLANNER):
    let dict_pos
    let graph
    let list_poi
    let pos_to_num
    let num_to_pos
    let tracker
    
    
    def __init__(self, roads, pinterests ):
        self.dict_pos =  HashTable((2 * roads.len()), make_sbox_hash())
        self.graph = WuGraph((2 * roads.len()))
        self.list_poi = pinterests
        self.pos_to_num = HashTable((2 * roads.len()), make_sbox_hash())
        self.num_to_pos = HashTable((2 * roads.len()), make_sbox_hash())
        self.tracker = 0
        
        for i in roads:
            let pos1 = [i[0], i[1]]
            let pos2 = [i[2], i[3]]
            
            if not self.dict_pos.mem?(pos1):
                self.dict_pos.put(pos1,None)
                self.pos_to_num.put(pos1, self.tracker)
                self.num_to_pos.put(self.tracker, pos1)
                self.tracker = self.tracker + 1
                 
            if not self.dict_pos.mem?(pos2):
                self.dict_pos.put(pos2, None)
                self.pos_to_num.put(pos2, self.tracker)
                self.num_to_pos.put(self.tracker, pos2)
                self.tracker = self.tracker + 1
                
            self.graph.set_edge(self.pos_to_num.get(pos1), self.pos_to_num.get(pos2), distance_squ(pos1,pos2))
        
        for i in pinterests:
             let pos = [i[0], i[1]]   
             let b = self.dict_pos.get(pos)     
             b = cons(i, b)             
             self.dict_pos.put(pos, b)
             
                
    def find_name(self,dst_name):
        for i in self.list_poi:
            if i[3] == dst_name:           
                return [i[0], i[1]]
                         
                
    def locate_all(self, dst_cat:  Cat? )->ListC[RawPos?]:
    
        let ans = None
        let detector = [True; self.tracker]
        
        for i in self.list_poi:            
            let a = [i[0], i[1]]           
            #println(self.pos_to_num.get(a))
            
            if ( detector[self.pos_to_num.get(a)]) and (i[2] == dst_cat):            
                #print("was here")
                detector[self.pos_to_num.get(a)] = False        
                ans = cons(a,ans)
            
        return ans
       
       
    def dts(self, a):
        let distance = [inf; (self.tracker+1)]
        let pred = [None; (self.tracker+1)]
        pred[a] = a
        let done = [False; (self.tracker+1)]
        distance[a] = 0
        
        def helper_comp(a, b):
            if distance[a] <= distance[b]:
                return True
            else:
                return False
           
        let todo = BinHeap(10, helper_comp)
        todo.insert(a)
    
        while (todo.len() != 0):
            let tracker = todo.find_min()
            todo.remove_min()
           
            if not done[tracker]:      
                done[tracker] = True 
                let mya = self.graph.get_adjacent(tracker)
            
                while mya != None:
                    if (distance[tracker] + self.graph.get_edge(tracker, mya.data)) < distance[mya.data]:                   
                        distance[mya.data] = distance[tracker] + self.graph.get_edge(tracker, mya.data)            
                        pred[mya.data] = tracker
       
                    todo.insert(mya.data) 
                    mya = mya.next
                 
        println(pred)
        println(distance)
      
        return [distance,pred]
    
        
    def plan_route(self, src_lat:  Lat?, src_lon:  Lon?,dst_name: Name?)->ListC[RawPos?]:
        let target = self.find_name(dst_name)
        if target == None: return None
        let target_num = self.pos_to_num.get(target)
         
        let preds = self.dts(self.pos_to_num.get([src_lat, src_lon]))
        preds = preds[1]
       
        if preds[target_num] == None:
            return None
        
        def findpath(vect, a, b):
            let tracker = vect[b]
            let s = ListQueue()
             
            if tracker == None: return []
  
            s.enqueue( b)
    
            while (tracker != a) :       
                s.enqueue(tracker)
                tracker = vect[tracker]
                  
            let ans = None
  
            while not s.empty?():
                ans = cons(self.num_to_pos.get(s.dequeue()), ans)
            
            if self.num_to_pos.get(a) !=  ans.data:
                ans = cons( self.num_to_pos.get(a), ans)      
                
            return ans
        
        return findpath(preds,self.pos_to_num.get([src_lat, src_lon]), target_num)  
        
              
    def find_nearby(self, src_lat:  Lat?,src_lon:  Lon?, dst_cat:  Cat?,n: nat?) -> ListC[RawPOI?]:
        #let start = self.pos_to_num.get([src_lat, src_lon]).poi
        
        let start = [src_lat, src_lon]
       
        let q = ListStack()
        let distances = self.dts(self.pos_to_num.get(start))[0]
        
        let pos = [i for i in range(distances.len()-1)]
        
        def compare(a, b):
            if distances[a] < distances[b]:
                return True
            else: 
                return False
                      
        heap_sort(pos, compare)
        let num = 0
        let i = 0
        let ans = None
        
        while (num < n) and (i < distances.len()-1):     
            let cat_name = (self.dict_pos.get(self.num_to_pos.get(pos[i])))   
            let b = cat_name
            
            while (b != None) and (num < n):
                if b.data[2] == dst_cat:
                    ans = cons(b.data, ans)
                    num = num + 1   
                b = b.next
                
            i = i + 1 
            
        return ans
        
                       
                        

def example_from_handout():
    return TripPlanner([[0,0, 0, 1],[0,1, 0,2],[0,0, 1,0], [1,0, 1,1], [0,1, 1,1], [0,2, 1,2],[1,1, 1,2], [1,2, 1, 3], [1,3, -0.2, 3.3]],
                       [[0,0, "food", "Sandwiches"], [0,1, "food", "Pasta"],[1,1, "bank", "Local Credit Union",], [1, 3,"bar", "Bar None"], [1, 3,"bar", "H Bar"], [-0.2, 3.3,"food", "Burritos"]])

def new_example():
    return TripPlanner([[0,0, 0, 1],[0,1, 0,2],[0,0, 1,0], [1,0, 1,1], [0,1, 1,1], [0,2, 1,2],[1,1, 1,2], [1,3, -0.2, 3.3]],
                       [[0,0, "food", "Sandwiches"], [0,1, "food", "Pasta"],[1,1, "bank", "Local Credit Union",], [1, 3,"bar", "Bar None"], [1, 3,"bar", "H Bar"], [-0.2, 3.3,"food", "Burritos"]])     
     
let t = TripPlanner([[0,0, 0, 1],[0,1, 0,2],[0,0, 1,0], [1,0, 1,1], [0,1, 1,1], [0,2, 1,2],[1,1, 1,2], [1,2,1,3], [1,3, -0.2, 3.3]],
                       [[0,0, "food", "sandwiches"], [0,1, "food", "pasta"],[1,3,"bar","bar none"], [1,3,"bar", "h bar"],[1,1, "bank", "local credit union",], [1, 3,"bar", "Bar None"], [1, 3,"bar", "H Bar"], [-0.2, 3.3,"food", "Bburritos"]])     
              
println(t.plan_route(1,1,"sandwiches"))                       
                       
                       
                        
 
#### ^^^ YOUR CODE HERE
