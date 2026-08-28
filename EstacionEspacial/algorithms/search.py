from algorithms.problems import SearchProblem
import algorithms.utils as utils
from world.game import Directions
from algorithms.heuristics import nullHeuristic


def tinyDiagnosticSearch(problem: SearchProblem):
    """
    Returns a hard-coded sequence of moves for the tinyDiagnostic layout.
    For any other station layout, the sequence of moves will be incorrect.
    """
    s = Directions.SOUTH
    e = Directions.EAST
    return [s, e, s, e, e, e, e, s, e, e, s, s, e, s, s, e, s, e, e, e, e, e, e, e]


def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    
    # stack se rige por Last In, First Out
    stack = utils.Stack()
    visitados = set()
    
    nodoInicial = problem.getStartState()
    stack.push((nodoInicial, []))
    
    while not stack.isEmpty():
        estadoActual, acciones = stack.pop()
        
        if problem.isGoalState(estadoActual):
            return acciones
        
        if estadoActual not in visitados:
            visitados.add(estadoActual)
            
        for siguienteEstado, accion, _ in problem.getSuccessors(estadoActual):
            if siguienteEstado not in visitados:
                agregarAccion = acciones + [accion]
                stack.push((siguienteEstado, agregarAccion))

    return []
    


def breadthFirstSearch(problem: SearchProblem):
    """
    Search the shallowest nodes in the search tree first.
    """
    
    # queue se rige por First In, First Out
    queue = utils.Queue()
    visitados = set()
    
    nodoInicial = problem.getStartState()
    queue.push((nodoInicial, []))
    
    while not queue.isEmpty():
        estadoActual, acciones = queue.pop()
        
        if problem.isGoalState(estadoActual):
            return acciones
        
        if estadoActual not in visitados:
            visitados.add(estadoActual)

        for siguienteEstado, accion, _ in problem.getSuccessors(estadoActual):
            if siguienteEstado not in visitados:
                agregarAccion = acciones + [accion]
                queue.push((siguienteEstado, agregarAccion))
                
    return []



def uniformCostSearch(problem: SearchProblem):
    """
    Search the node of least total cost first.
    """
    
    frontera = utils.PriorityQueue()
    estadoInicial = problem.getStartState()

    frontera.push((estadoInicial, [], 0), 0)

    visitados = set()

    while not frontera.isEmpty():
        estado, acciones, costo = frontera.pop()

        if estado not in visitados:

            visitados.add(estado)

            if problem.isGoalState(estado):
                return acciones

            for siguienteEstado, accion, costoPaso in problem.getSuccessors(estado):
                if siguienteEstado not in visitados:
                    nuevoCosto = costo + costoPaso
                    nuevasAcciones = acciones + [accion]

                    frontera.push((siguienteEstado, nuevasAcciones, nuevoCosto), nuevoCosto)

    return []


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """
    frontera = utils.PriorityQueue()
    estadoInicial = problem.getStartState()

    frontera.push((estadoInicial, [], 0), heuristic(estadoInicial, problem))

    visitados = set()

    while not frontera.isEmpty():
        estado, acciones, costo = frontera.pop()

        if estado not in visitados:

            visitados.add(estado)

            if problem.isGoalState(estado):
                return acciones

            for siguienteEstado, accion, costoPaso in problem.getSuccessors(estado):
                if siguienteEstado not in visitados:
                    nuevoCosto = costo + costoPaso
                    nuevasAcciones = acciones + [accion]

                    frontera.push((siguienteEstado, nuevasAcciones, nuevoCosto), nuevoCosto + heuristic(siguienteEstado, problem))

    return []

# Abbreviations (you can use them for the -f option in main.py)
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
