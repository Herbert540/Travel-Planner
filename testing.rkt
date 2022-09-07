#lang dssl2
import 'project-lib/graph.rkt'
import cons
import 'project-lib/binheap.rkt'
import 'project-lib/stack-queue.rkt'


let u =  WuGraph(5)
u.set_edge(0,1,5)
u.set_edge(0,3, 1)
u.set_edge(1,3, 3)
u.set_edge(1,4,10)
u.set_edge(1,2,4)
u.set_edge(2,4, 2)

def dts(graph, a):
    let distance = [inf; 5]
    
    let pred = [None; 5]
    
    pred[a] = a
    
    let done = [False; 5]
    
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
            
            
            let mya = graph.get_adjacent(tracker)
            
            while mya != None:
            
                
                if (distance[tracker] + graph.get_edge(tracker, mya.data)) < distance[mya.data]:
                    
                    
                    distance[mya.data] = distance[tracker] + graph.get_edge(tracker, mya.data)
                    
                    
                    pred[mya.data] = tracker
                    
                    
                todo.insert(mya.data)
                
                mya = mya.next
                 
                
    return [distance,pred]
      
def findpath(vect, a, b):
    let tracker = vect[1][b]
    let s = ListStack()
    
    while tracker != a:
        s.push(tracker)
        tracker = vect[1][tracker]
        
    let ans = None
    
    while not s.empty?():
        
        ans = cons(s.pop(), ans)
        
        
    return ans
    
    
let d = dts(u, 0)

findpath(d, 0, 4)
                    
             
                    
                    
    
    
    
    
    
    