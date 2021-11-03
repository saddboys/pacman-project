Project for Artificial Intelligence  

# commands

python pacman.py  
python pacman.py -l tinyMaze -p AStarFoodSearchAgent  
python pacman.py -l mediumMaze -p AStarFoodSearchAgent  
python pacman.py -l bigMaze -p AStarFoodSearchAgent  
python pacman.py -l testSearch -p AStarFoodSearchAgent  
python pacman.py -l trickySearch -p AStarFoodSearchAgent  
python pacman.py -l bigSearch  -p AStarFoodSearchAgent  

Agent:  
AStarFoodSearchAgent   
Takes a heuristic value calculated by taking the max cost between any two foods, and uses that heuristic for A* search  
This search agent is optimal, so will always find the least cost path for the pacman to eat all foods!  



